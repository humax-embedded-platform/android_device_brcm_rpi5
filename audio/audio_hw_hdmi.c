/*
 * Copyright (C) 2016 The Android Open Source Project
 * Copyright (C) 2021-2023 KonstaKANG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "audio_hw_rpi_hdmi"
#define LOG_NDEBUG 0

/* Rate-limited log: prints every N calls. Use for hot-path functions. */
#define ALOG_RATELIMIT(n, tag, fmt, ...) \
    do { \
        static unsigned _rl_ctr = 0; \
        if ((_rl_ctr++ % (n)) == 0) \
            ALOGD("[%s #%u] " fmt, tag, _rl_ctr, ##__VA_ARGS__); \
    } while (0)

#include <errno.h>
#include <fcntl.h>
#include <malloc.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdlib.h>
#include <unistd.h>

#include <log/log.h>
#include <cutils/str_parms.h>
#include <cutils/properties.h>

#include <hardware/hardware.h>
#include <system/audio.h>
#include <hardware/audio.h>

#include <alsa/asoundlib.h>
#include <audio_utils/resampler.h>
#include <audio_utils/echo_reference.h>
#include <hardware/audio_effect.h>
#include <audio_effects/effect_aec.h>


/* Minimum granularity - Arbitrary but small value */
#define CODEC_BASE_FRAME_COUNT 32

/* HU mic injection via named FIFO --------------------------------------- */
#define HU_MIC_FIFO_PATH    "/dev/hu_mic"
/* Ring buffer: power-of-2, holds ~16 periods of 24 kHz mono input.
 * HU mic: 24 kHz mono s16-LE = 2 bytes/sample.
 * One 48 kHz output period (PERIOD_SIZE frames, stereo) needs
 * PERIOD_SIZE/2 input samples = PERIOD_SIZE bytes from this buffer. */
#define HU_MIC_BUF_BITS     14
#define HU_MIC_BUF_SIZE     (1u << HU_MIC_BUF_BITS)  /* 16384 bytes */
#define HU_MIC_BUF_MASK     (HU_MIC_BUF_SIZE - 1u)
/* ------------------------------------------------------------------- */

/* number of base blocks in a short period (low latency) */
#define PERIOD_MULTIPLIER 32  /* 21 ms */
/* number of frames per short period (low latency) */
#define PERIOD_SIZE (CODEC_BASE_FRAME_COUNT * PERIOD_MULTIPLIER)
/* number of pseudo periods for low latency playback */
#define PLAYBACK_PERIOD_COUNT 4
#define PLAYBACK_PERIOD_START_THRESHOLD 2
#define CODEC_SAMPLING_RATE 48000
#define CHANNEL_STEREO 2
#define MIN_WRITE_SLEEP_US      5000

char device_name[PROPERTY_VALUE_MAX];

struct alsa_audio_device {
    struct audio_hw_device hw_device;

    pthread_mutex_t lock;   /* see note below on mutex acquisition order */
    int devices;
    struct alsa_stream_in *active_input;
    struct alsa_stream_out *active_output;
    bool mic_mute;

    /* HU mic FIFO injection */
    int      hu_mic_fd;
    uint8_t  hu_mic_buf[HU_MIC_BUF_SIZE];
    uint32_t hu_mic_wr;
    uint32_t hu_mic_rd;
};

struct alsa_stream_in {
    struct audio_stream_in stream;

    pthread_mutex_t lock;
    /* Mic input pipeline is purely the HU FIFO injection — no ALSA capture
     * device is opened here (the HDMI variant has no analog mic). Values
     * mirror what the HU mic patch in audio_hw.c upsamples to. */
    uint32_t sample_rate;   /* 48000 */
    uint32_t channels;      /* 2 (stereo) */
    size_t   period_size;   /* PERIOD_SIZE */
    bool unavailable;
    int standby;
    struct alsa_audio_device *dev;
    unsigned int read_frames;
    audio_devices_t device;
    int dump_fd;            /* -1 when disabled; toggled by prop audio_hal_dump */
};

struct alsa_stream_out {
    struct audio_stream_out stream;
    struct alsa_audio_device *dev;

    pthread_mutex_t lock;   /* see note below on mutex acquisition order */
    snd_pcm_t *pcm;

    snd_pcm_uframes_t period_size;
    unsigned int periods;
    snd_pcm_uframes_t buffer_size;

    bool unavailable;
    int standby;
    snd_pcm_uframes_t written;
};

static void get_alsa_device_name(char *name) {
    char hdmi_device[PROPERTY_VALUE_MAX];
    property_get("persist.audio.hdmi.device", hdmi_device, "vc4hdmi0");

    // use card configured in vc4-hdmi.conf to get IEC958 subframe conversion
    sprintf(name, "default:CARD=%s", hdmi_device);
}

/* must be called with hw device and output stream mutexes locked */
static int start_output_stream(struct alsa_stream_out *out)
{
    struct alsa_audio_device *adev = out->dev;

    if (out->unavailable)
        return -ENODEV;

    ALOGI("start_output_stream: %s", device_name);

    int r;
    snd_pcm_t *pcm;

    if ((r = snd_pcm_open(&pcm, device_name, SND_PCM_STREAM_PLAYBACK, 0) < 0)) {
        ALOGE("cannot open pcm_out driver: %s", snd_strerror(r));
        adev->active_output = NULL;
        out->unavailable = true;
        return -ENODEV;
    }
    out->pcm = pcm;

    snd_pcm_hw_params_t *hwp;
    snd_pcm_hw_params_alloca(&hwp);
    snd_pcm_hw_params_any(pcm, hwp);
    snd_pcm_hw_params_set_access(pcm, hwp, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm, hwp, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_rate(pcm, hwp, CODEC_SAMPLING_RATE, 0);
    snd_pcm_hw_params_set_channels(pcm, hwp, CHANNEL_STEREO);

    // Configurue period_size, periods and buffer_size
    int dir = 0;
    out->period_size = PERIOD_SIZE;
    if ((r = snd_pcm_hw_params_set_period_size_near(pcm, hwp, &out->period_size, &dir)) < 0) {
        ALOGE("cannot snd_pcm_hw_params_set_period_size_near: %s", snd_strerror(r));
        adev->active_output = NULL;
        out->unavailable = true;
        return -ENODEV;
    }
    dir = 0;
    out->periods = PLAYBACK_PERIOD_COUNT;
    if ((r = snd_pcm_hw_params_set_periods_near(pcm, hwp, &out->periods, &dir)) < 0) {
        ALOGE("cannot snd_pcm_hw_params_set_periods_near: %s", snd_strerror(r));
        adev->active_output = NULL;
        out->unavailable = true;
        return -ENODEV;
    }
    out->buffer_size = out->period_size * out->periods;
    if ((r = snd_pcm_hw_params_set_buffer_size_near(pcm, hwp, &out->buffer_size)) < 0) {
        ALOGE("cannot snd_pcm_hw_params_set_buffer_size_near: %s", snd_strerror(r));
        adev->active_output = NULL;
        out->unavailable = true;
        return -ENODEV;
    }

    //write the hw params
    if ((r = snd_pcm_hw_params(pcm, hwp)) < 0) {
        ALOGE("cannot snd_pcm_hw_params: %s", snd_strerror(r));
        adev->active_output = NULL;
        out->unavailable = true;
        return -ENODEV;
    }

    //Software parameters
    snd_pcm_sw_params_t *swp;
    snd_pcm_sw_params_alloca(&swp);
    snd_pcm_sw_params_current(pcm, swp);

    // set avail_min to period_size
    if ((r = snd_pcm_sw_params_set_avail_min(pcm, swp, out->period_size)) < 0) {
        ALOGE("cannot snd_pcm_sw_params_set_avail_min: %s", snd_strerror(r));
        adev->active_output = NULL;
        out->unavailable = true;
        return -ENODEV;
    }
    // set start_threshold to period_size * PLAYBACK_PERIOD_START_THRESHOLD
    if ((r = snd_pcm_sw_params_set_start_threshold(pcm, swp, out->period_size * PLAYBACK_PERIOD_START_THRESHOLD)) < 0) {
        ALOGE("cannot snd_pcm_sw_params_set_start_threshold: %s", snd_strerror(r));
        adev->active_output = NULL;
        out->unavailable = true;
        return -ENODEV;
    }
    //write the sw params
    if ((r = snd_pcm_sw_params(pcm, swp)) < 0) {
        ALOGE("cannot snd_pcm_sw_params: %s", snd_strerror(r));
        adev->active_output = NULL;
        out->unavailable = true;
        return -ENODEV;
    }

    // prepare
    if ((r = snd_pcm_prepare(pcm)) < 0) {
        ALOGE("cannot snd_pcm_prepare: %s", snd_strerror(r));
        adev->active_output = NULL;
        out->unavailable = true;
        return -ENODEV;
    }

    adev->active_output = out;
    return 0;
}

static uint32_t out_get_sample_rate(const struct audio_stream *stream)
{
    ALOGV("out_get_sample_rate: %d", CODEC_SAMPLING_RATE);
    return CODEC_SAMPLING_RATE;
}

static int out_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    ALOGV("out_set_sample_rate: %d", 0);
    return -ENOSYS;
}

static size_t out_get_buffer_size(const struct audio_stream *stream)
{
    struct alsa_stream_out *out = (struct alsa_stream_out *)stream;

    /* return the closest majoring multiple of 16 frames, as
     * audioflinger expects audio buffers to be a multiple of 16 frames */
    size_t size = out->period_size;
    size = ((size + 15) / 16) * 16;
    ALOGV("out_get_buffer_size: %ld", (long int)size);
    return size * audio_stream_out_frame_size((struct audio_stream_out *)stream);
}

static audio_channel_mask_t out_get_channels(const struct audio_stream *stream)
{
    ALOGV("out_get_channels: %d", CHANNEL_STEREO);
    return audio_channel_out_mask_from_count(CHANNEL_STEREO);
}

static audio_format_t out_get_format(const struct audio_stream *stream)
{
    ALOGV("out_get_format: %d", AUDIO_FORMAT_PCM_16_BIT);
    return AUDIO_FORMAT_PCM_16_BIT;
}

static int out_set_format(struct audio_stream *stream, audio_format_t format)
{
    ALOGV("out_set_format: %d",format);
    return -ENOSYS;
}

static int do_output_standby(struct alsa_stream_out *out)
{
    struct alsa_audio_device *adev = out->dev;

    if (!out->standby) {
        snd_pcm_close(out->pcm);
        out->pcm = NULL;
        adev->active_output = NULL;
        out->standby = 1;
    }
    return 0;
}

static int out_standby(struct audio_stream *stream)
{
    ALOGV("out_standby");
    struct alsa_stream_out *out = (struct alsa_stream_out *)stream;
    int status;

    pthread_mutex_lock(&out->dev->lock);
    pthread_mutex_lock(&out->lock);
    status = do_output_standby(out);
    pthread_mutex_unlock(&out->lock);
    pthread_mutex_unlock(&out->dev->lock);
    return status;
}

static int out_dump(const struct audio_stream *stream, int fd)
{
    ALOGV("out_dump");
    return 0;
}

static int out_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    ALOGV("out_set_parameters");
    struct alsa_stream_out *out = (struct alsa_stream_out *)stream;
    struct alsa_audio_device *adev = out->dev;
    struct str_parms *parms;
    char value[32];
    int val = 0;
    int ret = -EINVAL;

    if (kvpairs == NULL || kvpairs[0] == 0) {
        return 0;
    }

    parms = str_parms_create_str(kvpairs);

    if (str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_ROUTING, value, sizeof(value)) >= 0) {
        val = atoi(value);
        pthread_mutex_lock(&adev->lock);
        pthread_mutex_lock(&out->lock);
        if (((adev->devices & AUDIO_DEVICE_OUT_ALL) != val) && (val != 0)) {
            adev->devices &= ~AUDIO_DEVICE_OUT_ALL;
            adev->devices |= val;
        }
        pthread_mutex_unlock(&out->lock);
        pthread_mutex_unlock(&adev->lock);
        ret = 0;
    }

    str_parms_destroy(parms);
    return ret;
}

static char * out_get_parameters(const struct audio_stream *stream, const char *keys)
{
    ALOGV("out_get_parameters");
    return strdup("");
}

static uint32_t out_get_latency(const struct audio_stream_out *stream)
{
    ALOGV("out_get_latency");
    struct alsa_stream_out *out = (struct alsa_stream_out *)stream;
    // latency = buffer_size / rate
    return (out->buffer_size * 1000) / CODEC_SAMPLING_RATE;
}

static int out_set_volume(struct audio_stream_out *stream, float left,
        float right)
{
    ALOGV("out_set_volume: Left:%f Right:%f", left, right);
    return 0;
}

static ssize_t out_write(struct audio_stream_out *stream, const void* buffer,
        size_t bytes)
{
    int ret;
    struct alsa_stream_out *out = (struct alsa_stream_out *)stream;
    struct alsa_audio_device *adev = out->dev;
    size_t frame_size = audio_stream_out_frame_size(stream);
    snd_pcm_uframes_t out_frames = bytes / frame_size;

    /* acquiring hw device mutex systematically is useful if a low priority thread is waiting
     * on the output stream mutex - e.g. executing select_mode() while holding the hw device
     * mutex
     */
    pthread_mutex_lock(&adev->lock);
    pthread_mutex_lock(&out->lock);
    if (out->standby) {
        ret = start_output_stream(out);
        if (ret != 0) {
            pthread_mutex_unlock(&adev->lock);
            goto exit;
        }
        out->standby = 0;
    }

    pthread_mutex_unlock(&adev->lock);

    ALOGV("out_write: out_frames:%ld", (long int)out_frames);

    ret = snd_pcm_writei(out->pcm, buffer, out_frames);
    if (ret == out_frames) {
        out->written += out_frames;
    }
exit:
    pthread_mutex_unlock(&out->lock);

    if (ret != out_frames) {
        if (ret == -EPIPE) {
            ALOGE("underrun deteced -> redo snd_pcm_prepare");
            snd_pcm_prepare(out->pcm);
        } else {
            ALOGE("out_write err: %s", snd_strerror(ret));
            usleep((int64_t)bytes * 1000000 / audio_stream_out_frame_size(stream) /
                out_get_sample_rate(&stream->common));
        }
    }

    return bytes;
}

static int out_get_render_position(const struct audio_stream_out *stream,
        uint32_t *dsp_frames)
{
    *dsp_frames = 0;
    ALOGV("out_get_render_position: dsp_frames: %p", dsp_frames);
    return -EINVAL;
}

static int out_get_presentation_position(const struct audio_stream_out *stream,
                                   uint64_t *frames, struct timespec *timestamp)
{
    struct alsa_stream_out *out = (struct alsa_stream_out *)stream;
    int ret = -1;

    if (out->pcm) {
        snd_pcm_uframes_t avail;
        int r;
        if ((r = snd_pcm_htimestamp(out->pcm, &avail, timestamp)) == 0) {
            int64_t signed_frames = (int64_t)(out->written) - out->buffer_size + avail;
            if (signed_frames >= 0) {
                *frames = signed_frames;
                ret = 0;
            }
            ALOGV("out_get_presentation_position: %ld", (long int)(*frames));
        } else {
            ALOGE("out_get_presentation_position: err: %s", snd_strerror(r));
        }
    } else {
        ALOGV("out_get_presentation_position: stream in standby");
    }
    return ret;
}


static int out_add_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    ALOGV("out_add_audio_effect: %p", effect);
    return 0;
}

static int out_remove_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    ALOGV("out_remove_audio_effect: %p", effect);
    return 0;
}

static int out_get_next_write_timestamp(const struct audio_stream_out *stream,
        int64_t *timestamp)
{
    *timestamp = 0;
    ALOGV("out_get_next_write_timestamp: %ld", (long int)(*timestamp));
    return -EINVAL;
}

/* HU mic helpers — ported from audio_hw.c (rpi variant). The HDMI variant
 * has no ALSA capture device, so the input pipeline is purely the FIFO
 * injection: in_read drains /dev/hu_mic into a ring buffer and upsamples
 * 24 kHz mono → 48 kHz stereo for AudioFlinger.
 */
static void in_dump_write(struct alsa_stream_in *in, const void *buf, size_t bytes);
static int do_input_standby(struct alsa_stream_in *in);

static int start_input_stream(struct alsa_stream_in *in) {
    struct alsa_audio_device *adev = in->dev;

    ALOGD("start_input_stream: device=0x%08x unavailable=%d standby=%d",
          in->device, in->unavailable, in->standby);

    if (in->unavailable) {
        return -ENODEV;
    }

    ALOGD("start_input_stream: HU mic path, current fd=%d", adev->hu_mic_fd);
    if (adev->hu_mic_fd < 0) {
        adev->hu_mic_fd = open(HU_MIC_FIFO_PATH, O_RDONLY | O_NONBLOCK);
        if (adev->hu_mic_fd < 0)
            ALOGE("start_input_stream: failed to open %s: %s",
                  HU_MIC_FIFO_PATH, strerror(errno));
        else
            ALOGI("start_input_stream: opened HU mic FIFO %s fd=%d",
                  HU_MIC_FIFO_PATH, adev->hu_mic_fd);
    } else {
        ALOGD("start_input_stream: HU mic FIFO already open fd=%d", adev->hu_mic_fd);
    }
    ALOGD("start_input_stream: ring buf wr=%u rd=%u avail=%u",
          adev->hu_mic_wr, adev->hu_mic_rd,
          adev->hu_mic_wr - adev->hu_mic_rd);

    adev->active_input = in;
    return 0;
}

/** audio_stream_in implementation **/
static uint32_t in_get_sample_rate(const struct audio_stream *stream)
{
    struct alsa_stream_in *in = (struct alsa_stream_in *)stream;
    return in->sample_rate ? in->sample_rate : 48000;
}

static int in_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    ALOGV("in_set_sample_rate: %d", rate);
    return -ENOSYS;
}

static size_t in_get_buffer_size(const struct audio_stream *stream)
{
    struct alsa_stream_in *in = (struct alsa_stream_in *)stream;
    size_t period = in->period_size ? in->period_size : PERIOD_SIZE;
    size_t size = ((period + 15) / 16) * 16;
    return size * audio_stream_in_frame_size((struct audio_stream_in *)stream);
}

static audio_channel_mask_t in_get_channels(const struct audio_stream *stream)
{
    struct alsa_stream_in *in = (struct alsa_stream_in *)stream;
    return audio_channel_in_mask_from_count(in->channels ? in->channels : 2);
}

static audio_format_t in_get_format(const struct audio_stream *stream)
{
    return AUDIO_FORMAT_PCM_16_BIT;
}

static int in_set_format(struct audio_stream *stream, audio_format_t format)
{
    return -ENOSYS;
}

static int in_standby(struct audio_stream *stream)
{
    struct alsa_stream_in *in = (struct alsa_stream_in *)stream;
    pthread_mutex_lock(&in->dev->lock);
    pthread_mutex_lock(&in->lock);
    int status = do_input_standby(in);
    pthread_mutex_unlock(&in->lock);
    pthread_mutex_unlock(&in->dev->lock);
    return status;
}

static int in_dump(const struct audio_stream *stream, int fd)
{
    return 0;
}

static int in_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    return 0;
}

static char * in_get_parameters(const struct audio_stream *stream,
        const char *keys)
{
    return strdup("");
}

static int in_set_gain(struct audio_stream_in *stream, float gain)
{
    return 0;
}

static ssize_t in_read(struct audio_stream_in *stream, void* buffer,
        size_t bytes)
{
    struct alsa_stream_in *in = (struct alsa_stream_in *)stream;
    struct alsa_audio_device *adev = in->dev;
    size_t frame_size = audio_stream_in_frame_size(stream);
    ssize_t ret = 0;
    ALOG_RATELIMIT(300, "in_read", "bytes=%zu", bytes);
    pthread_mutex_lock(&adev->lock);
    pthread_mutex_lock(&in->lock);

    if (in->standby) {
        ret = start_input_stream(in);
        if (ret != 0) {
            pthread_mutex_unlock(&adev->lock);
            goto exit;
        }
        in->standby = 0;
    }
    pthread_mutex_unlock(&adev->lock);

    size_t in_frames = bytes / frame_size;

    /* ---- HU mic injection ------------------------------------------ */
    /* Step 1: Drain the FIFO into the ring buffer (non-blocking). */
    if (adev->hu_mic_fd >= 0) {
        uint8_t tmp[512];
        ssize_t n;
        while (1) {
            uint32_t avail_space = HU_MIC_BUF_SIZE -
                    (adev->hu_mic_wr - adev->hu_mic_rd);
            if (avail_space == 0) break;
            size_t to_read = avail_space < sizeof(tmp) ? avail_space : sizeof(tmp);
            n = read(adev->hu_mic_fd, tmp, to_read);
            if (n <= 0) {
                if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    /* Writer disconnected — try to reopen for next connection */
                    close(adev->hu_mic_fd);
                    adev->hu_mic_fd = open(HU_MIC_FIFO_PATH,
                                        O_RDONLY | O_NONBLOCK);
                    ALOGD("hu_mic: pipe closed, reopened fd=%d", adev->hu_mic_fd);
                }
                break;
            }
            for (ssize_t i = 0; i < n; i++)
                adev->hu_mic_buf[(adev->hu_mic_wr++) & HU_MIC_BUF_MASK] = tmp[i];
        }
    }

    /* Step 2: If ring buffer has >= in_frames bytes of 24 kHz mono,
     * upsample to 48 kHz stereo and overwrite the ALSA buffer. */
    uint32_t hu_avail = adev->hu_mic_wr - adev->hu_mic_rd;
    ALOG_RATELIMIT(100, "in_read",
                "hu_mic fd=%d avail=%u in_frames=%zu buf_fill=%.1f%%",
                adev->hu_mic_fd, hu_avail, in_frames,
                100.0f * hu_avail / HU_MIC_BUF_SIZE);
    if (hu_avail >= (uint32_t)in_frames) {
        /* Each input sample (24 kHz mono) -> 2 output frames x 2 ch = 4 s16s */
        int16_t *out16 = (int16_t *)buffer;
        size_t input_samples = in_frames / 2; /* = in_frames/2 @ 24kHz */
        for (size_t i = 0; i < input_samples; i++) {
            uint8_t lo = adev->hu_mic_buf[(adev->hu_mic_rd++) & HU_MIC_BUF_MASK];
            uint8_t hi = adev->hu_mic_buf[(adev->hu_mic_rd++) & HU_MIC_BUF_MASK];
            int16_t s = (int16_t)(lo | ((uint16_t)hi << 8));
            /* Gain boost: HU mic signal is very quiet (~1% full-scale).
             * Amplify 20x (~26 dB) to bring speech into recognisable range. */
            int32_t s_amp = (int32_t)s * 20;
            if      (s_amp >  32767) s_amp =  32767;
            else if (s_amp < -32768) s_amp = -32768;
            s = (int16_t)s_amp;
            /* Duplicate to 2 output frames (zero-order hold), stereo */
            size_t j = i * 4;
            out16[j + 0] = s;  /* frame 0, L */
            out16[j + 1] = s;  /* frame 0, R */
            out16[j + 2] = s;  /* frame 1, L */
            out16[j + 3] = s;  /* frame 1, R */
        }
        ret = 0;
    } else {
        /* No FIFO data — exit path will sleep + zero the buffer. */
        ALOG_RATELIMIT(300, "in_read", "HU mic starved (avail=%u < need=%zu)",
                    hu_avail, in_frames);
        ret = -1;
    }
    /* For the success path only: pace to real time so we don't spin.
     * The starved path is already paced by the exit-block usleep below. */
    usleep((long)in_frames * 1000000L / CODEC_SAMPLING_RATE);

exit:
    pthread_mutex_unlock(&in->lock);

    if (ret != 0) {
        usleep((int64_t)bytes * 1000000 / frame_size /
                in_get_sample_rate(&stream->common));
        memset(buffer, 0, bytes);
    }

    in_dump_write(in, buffer, bytes);
    return bytes;
}

static void in_dump_write(struct alsa_stream_in *in, const void *buf, size_t bytes)
{
    char prop[PROPERTY_VALUE_MAX];
    property_get("audio_hal_dump", prop, "0");
    if (prop[0] == '1') {
        if (in->dump_fd < 0) {
            in->dump_fd = open("/data/misc/audioserver/hu_mic_dump.pcm",
                               O_WRONLY | O_CREAT | O_TRUNC, 0644);
            ALOGI("in_dump: started fd=%d", in->dump_fd);
        }
    } else if (in->dump_fd >= 0) {
        ALOGI("in_dump: stopped");
        close(in->dump_fd);
        in->dump_fd = -1;
    }
    if (in->dump_fd >= 0)
        write(in->dump_fd, buf, bytes);
}

static int do_input_standby(struct alsa_stream_in *in)
{
    if (!in->standby) {
        /* Drain FIFO and flush ring buffer */
        struct alsa_audio_device *adev = in->dev;
        size_t fifo_drained = 0;
        uint32_t ring_pending = 0;

        if (adev && adev->hu_mic_fd >= 0) {
            uint8_t sink[512];
            while (1) {
                ssize_t n = read(adev->hu_mic_fd, sink, sizeof(sink));
                if (n > 0) {
                    fifo_drained += (size_t)n;
                    /* Safety cap: do not spin forever if writer is flooding. */
                    if (fifo_drained >= (HU_MIC_BUF_SIZE * 8u)) {
                        break;
                    }
                    continue;
                }
                if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    ALOGW("hu_mic: FIFO discard read error: %s", strerror(errno));
                }
                break;
            }
            ring_pending = adev->hu_mic_wr - adev->hu_mic_rd;
            adev->hu_mic_wr = 0;
            adev->hu_mic_rd = 0;
            ALOGI("hu_mic: reset on mic standby (ring_pending=%u bytes, fifo_discarded=%zu bytes)",
                  ring_pending, fifo_drained);
        }

        /* Only clear active_input if it still points to this stream. */
        if (adev->active_input == in)
            adev->active_input = NULL;

        in->standby = 1;
    }
    return 0;
}

static uint32_t in_get_input_frames_lost(struct audio_stream_in *stream)
{
    return 0;
}

static int in_add_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

static int in_remove_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

static int adev_open_output_stream(struct audio_hw_device *dev,
        audio_io_handle_t handle,
        audio_devices_t devices,
        audio_output_flags_t flags,
        struct audio_config *config,
        struct audio_stream_out **stream_out,
        const char *address __unused)
{
    ALOGV("adev_open_output_stream...");

    struct alsa_audio_device *ladev = (struct alsa_audio_device *)dev;
    struct alsa_stream_out *out;
    int ret = 0;

    out = (struct alsa_stream_out *)calloc(1, sizeof(struct alsa_stream_out));
    if (!out)
        return -ENOMEM;

    out->stream.common.get_sample_rate = out_get_sample_rate;
    out->stream.common.set_sample_rate = out_set_sample_rate;
    out->stream.common.get_buffer_size = out_get_buffer_size;
    out->stream.common.get_channels = out_get_channels;
    out->stream.common.get_format = out_get_format;
    out->stream.common.set_format = out_set_format;
    out->stream.common.standby = out_standby;
    out->stream.common.dump = out_dump;
    out->stream.common.set_parameters = out_set_parameters;
    out->stream.common.get_parameters = out_get_parameters;
    out->stream.common.add_audio_effect = out_add_audio_effect;
    out->stream.common.remove_audio_effect = out_remove_audio_effect;
    out->stream.get_latency = out_get_latency;
    out->stream.set_volume = out_set_volume;
    out->stream.write = out_write;
    out->stream.get_render_position = out_get_render_position;
    out->stream.get_next_write_timestamp = out_get_next_write_timestamp;
    out->stream.get_presentation_position = out_get_presentation_position;

    out->period_size = PERIOD_SIZE;
    out->periods = PLAYBACK_PERIOD_COUNT;
    out->buffer_size = out->period_size * out->periods;

    out->dev = ladev;
    out->standby = 1;
    out->unavailable = false;

    config->format = out_get_format(&out->stream.common);
    config->channel_mask = out_get_channels(&out->stream.common);
    config->sample_rate = out_get_sample_rate(&out->stream.common);

    *stream_out = &out->stream;

    /* TODO The retry mechanism isn't implemented in AudioPolicyManager/AudioFlinger. */
    ret = 0;

    return ret;
}

static void adev_close_output_stream(struct audio_hw_device *dev,
        struct audio_stream_out *stream)
{
    ALOGV("adev_close_output_stream...");
    free(stream);
}

static int adev_set_parameters(struct audio_hw_device *dev, const char *kvpairs)
{
    ALOGV("adev_set_parameters");
    return -ENOSYS;
}

static char * adev_get_parameters(const struct audio_hw_device *dev,
        const char *keys)
{
    ALOGV("adev_get_parameters");
    return strdup("");
}

static int adev_init_check(const struct audio_hw_device *dev)
{
    ALOGV("adev_init_check");
    return 0;
}

static int adev_set_voice_volume(struct audio_hw_device *dev, float volume)
{
    ALOGV("adev_set_voice_volume: %f", volume);
    return -ENOSYS;
}

static int adev_set_master_volume(struct audio_hw_device *dev, float volume)
{
    ALOGV("adev_set_master_volume: %f", volume);
    return -ENOSYS;
}

static int adev_get_master_volume(struct audio_hw_device *dev, float *volume)
{
    ALOGV("adev_get_master_volume: %f", *volume);
    return -ENOSYS;
}

static int adev_set_master_mute(struct audio_hw_device *dev, bool muted)
{
    ALOGV("adev_set_master_mute: %d", muted);
    return -ENOSYS;
}

static int adev_get_master_mute(struct audio_hw_device *dev, bool *muted)
{
    ALOGV("adev_get_master_mute: %d", *muted);
    return -ENOSYS;
}

static int adev_set_mode(struct audio_hw_device *dev, audio_mode_t mode)
{
    ALOGV("adev_set_mode: %d", mode);
    return 0;
}

static int adev_set_mic_mute(struct audio_hw_device *dev, bool state)
{
    ALOGV("adev_set_mic_mute: %d",state);
    return -ENOSYS;
}

static int adev_get_mic_mute(const struct audio_hw_device *dev, bool *state)
{
    ALOGV("adev_get_mic_mute");
    return -ENOSYS;
}

static size_t adev_get_input_buffer_size(const struct audio_hw_device *dev __unused,
        const struct audio_config *config)
{
    size_t channels = audio_channel_count_from_in_mask(config->channel_mask);
    size_t bytes_per_sample = audio_bytes_per_sample(config->format);
    if (channels == 0) channels = CHANNEL_STEREO;
    if (bytes_per_sample == 0) bytes_per_sample = 2;
    size_t size = PERIOD_SIZE * channels * bytes_per_sample;
    /* round to a multiple of 16 frames as AudioFlinger expects */
    size = ((size + 15) / 16) * 16;
    ALOGV("adev_get_input_buffer_size: %zu (ch=%zu bps=%zu)",
          size, channels, bytes_per_sample);
    return size;
}

static int adev_open_input_stream(struct audio_hw_device *dev,
        audio_io_handle_t handle,
        audio_devices_t devices,
        struct audio_config *config,
        struct audio_stream_in **stream_in,
        audio_input_flags_t flags __unused,
        const char *address __unused,
        audio_source_t source __unused)
{
    struct alsa_audio_device *ladev = (struct alsa_audio_device *)dev;
    struct alsa_stream_in *in;

    ALOGI("adev_open_input_stream: devices=0x%08x req sr=%u ch=0x%x fmt=%d",
          devices, config->sample_rate, config->channel_mask, config->format);

    in = (struct alsa_stream_in *)calloc(1, sizeof(struct alsa_stream_in));
    if (!in)
        return -ENOMEM;

    in->stream.common.get_sample_rate = in_get_sample_rate;
    in->stream.common.set_sample_rate = in_set_sample_rate;
    in->stream.common.get_buffer_size = in_get_buffer_size;
    in->stream.common.get_channels = in_get_channels;
    in->stream.common.get_format = in_get_format;
    in->stream.common.set_format = in_set_format;
    in->stream.common.standby = in_standby;
    in->stream.common.dump = in_dump;
    in->stream.common.set_parameters = in_set_parameters;
    in->stream.common.get_parameters = in_get_parameters;
    in->stream.common.add_audio_effect = in_add_audio_effect;
    in->stream.common.remove_audio_effect = in_remove_audio_effect;
    in->stream.set_gain = in_set_gain;
    in->stream.read = in_read;
    in->stream.get_input_frames_lost = in_get_input_frames_lost;

    in->dev = ladev;
    in->standby = 1;
    in->unavailable = false;
    in->device = devices;
    in->sample_rate = CODEC_SAMPLING_RATE;          /* 48000 */
    in->channels = CHANNEL_STEREO;                  /* 2 */
    in->period_size = PERIOD_SIZE;
    in->dump_fd = -1;

    /* Force the framework to use the format the HU mic pipeline produces. */
    config->sample_rate = CODEC_SAMPLING_RATE;
    config->channel_mask = AUDIO_CHANNEL_IN_STEREO;
    config->format = AUDIO_FORMAT_PCM_16_BIT;

    *stream_in = &in->stream;
    return 0;
}

static void adev_close_input_stream(struct audio_hw_device *dev __unused,
        struct audio_stream_in *stream)
{
    struct alsa_stream_in *in = (struct alsa_stream_in *)stream;
    ALOGI("adev_close_input_stream");
    do_input_standby(in);
    if (in->dump_fd >= 0) {
        close(in->dump_fd);
        in->dump_fd = -1;
    }
    free(in);
}

static int adev_dump(const audio_hw_device_t *device, int fd)
{
    ALOGV("adev_dump");
    return 0;
}

static int adev_close(hw_device_t *device)
{
    struct alsa_audio_device *adev = (struct alsa_audio_device *)device;
    ALOGI("adev_close");
    if (adev) {
        if (adev->hu_mic_fd >= 0) {
            close(adev->hu_mic_fd);
            adev->hu_mic_fd = -1;
        }
    }
    free(device);
    return 0;
}

static int adev_open(const hw_module_t* module, const char* name,
        hw_device_t** device)
{
    struct alsa_audio_device *adev;

    ALOGV("adev_open: %s", name);

    get_alsa_device_name(device_name);
    ALOGI("adev_open: %s", device_name);

    if (strcmp(name, AUDIO_HARDWARE_INTERFACE) != 0)
        return -EINVAL;

    adev = calloc(1, sizeof(struct alsa_audio_device));
    if (!adev)
        return -ENOMEM;

    adev->hw_device.common.tag = HARDWARE_DEVICE_TAG;
    adev->hw_device.common.version = AUDIO_DEVICE_API_VERSION_2_0;
    adev->hw_device.common.module = (struct hw_module_t *) module;
    adev->hw_device.common.close = adev_close;
    adev->hw_device.init_check = adev_init_check;
    adev->hw_device.set_voice_volume = adev_set_voice_volume;
    adev->hw_device.set_master_volume = adev_set_master_volume;
    adev->hw_device.get_master_volume = adev_get_master_volume;
    adev->hw_device.set_master_mute = adev_set_master_mute;
    adev->hw_device.get_master_mute = adev_get_master_mute;
    adev->hw_device.set_mode = adev_set_mode;
    adev->hw_device.set_mic_mute = adev_set_mic_mute;
    adev->hw_device.get_mic_mute = adev_get_mic_mute;
    adev->hw_device.set_parameters = adev_set_parameters;
    adev->hw_device.get_parameters = adev_get_parameters;
    adev->hw_device.get_input_buffer_size = adev_get_input_buffer_size;
    adev->hw_device.open_output_stream = adev_open_output_stream;
    adev->hw_device.close_output_stream = adev_close_output_stream;
    adev->hw_device.open_input_stream = adev_open_input_stream;
    adev->hw_device.close_input_stream = adev_close_input_stream;
    adev->hw_device.dump = adev_dump;

    adev->devices = AUDIO_DEVICE_NONE;

    /* Open HU mic FIFO at HAL startup so the writer side (HuMicLocalBridge)
     * doesn't see EAGAIN before the first AudioRecord is created. The
     * reader-side fd is held until adev_close even when no input stream
     * is active — drain happens at standby/start. */
    adev->hu_mic_fd = open(HU_MIC_FIFO_PATH, O_RDONLY | O_NONBLOCK);
    if (adev->hu_mic_fd < 0) {
        ALOGW("adev_open: HU mic FIFO %s not openable yet (%s) — will retry on input start",
              HU_MIC_FIFO_PATH, strerror(errno));
    } else {
        ALOGI("adev_open: HU mic FIFO %s opened fd=%d",
              HU_MIC_FIFO_PATH, adev->hu_mic_fd);
    }
    adev->hu_mic_wr = 0;
    adev->hu_mic_rd = 0;

    *device = &adev->hw_device.common;

    return 0;
}

static struct hw_module_methods_t hal_module_methods = {
    .open = adev_open,
};

struct audio_module HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = AUDIO_MODULE_API_VERSION_0_1,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = AUDIO_HARDWARE_MODULE_ID,
        .name = "Raspberry Pi audio hdmi HW HAL",
        .author = "The Android Open Source Project",
        .methods = &hal_module_methods,
    },
};
