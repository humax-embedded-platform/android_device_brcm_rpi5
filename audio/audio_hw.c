/*
 * Copyright (C) 2016 The Android Open Source Project
 * Copyright (C) 2021-2022 KonstaKANG
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

#define LOG_TAG "audio_hw_rpi"
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

#include <sound/asound.h>
#include <tinyalsa/asoundlib.h>
#include <audio_utils/resampler.h>
#include <audio_utils/echo_reference.h>
#include <hardware/audio_effect.h>
#include <hardware/audio_alsaops.h>
#include <audio_effects/effect_aec.h>


/* Minimum granularity - Arbitrary but small value */
#define CODEC_BASE_FRAME_COUNT 32

/* HU mic injection via named FIFO --------------------------------------- */
#define HU_MIC_FIFO_PATH    "/dev/hu_mic"
/* Ring buffer: power-of-2, holds ~16 periods of 24 kHz mono input.
 * HU mic: 24 kHz mono s16-LE ΓåÆ 2 bytes/sample.
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

int pcm_card;
int pcm_device;

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

struct alsa_stream_out {
    struct audio_stream_out stream;

    pthread_mutex_t lock;   /* see note below on mutex acquisition order */
    struct pcm_config config;
    struct pcm *pcm;
    bool unavailable;
    int standby;
    struct alsa_audio_device *dev;
    int write_threshold;
    unsigned int written;
};

struct alsa_stream_in {
    struct audio_stream_in stream;

    pthread_mutex_t lock;   /* see note below on mutex acquisition order */
    struct pcm_config config;
    struct pcm *pcm;
    bool unavailable;
    int standby;
    struct alsa_audio_device *dev;
    unsigned int read_frames;
    audio_devices_t device;
};

struct pcm_config in_config = {
    .channels = CHANNEL_STEREO,
    .rate = 48000,
    .format = PCM_FORMAT_S16_LE,
    .period_size = PERIOD_SIZE,
    .period_count = 8,
    .start_threshold = 1,
};

static int get_pcm_card();
static int get_pcm_device();
static int do_input_standby(struct alsa_stream_in *in);
static int do_output_standby(struct alsa_stream_out *out);
static int get_input_pcm_device(audio_devices_t device);

static int get_input_pcm_card(audio_devices_t device)
{
    if ((device & AUDIO_DEVICE_IN_HDMI) == AUDIO_DEVICE_IN_HDMI) {
        return 3;
    }
    return get_pcm_card();
}

static int start_input_stream(struct alsa_stream_in *in) {
    struct alsa_audio_device *adev = in->dev;

    ALOGD("start_input_stream: device=0x%08x unavailable=%d standby=%d",
          in->device, in->unavailable, in->standby);

    if(in->unavailable) {
        return -ENODEV;
    }

    if ((in->device & AUDIO_DEVICE_IN_HDMI) == AUDIO_DEVICE_IN_HDMI) {
        int card = get_input_pcm_card(in->device);
        int device = get_input_pcm_device(in->device);

        ALOGD("start_input_stream: HDMI path -> card=%d device=%d "
              "rate=%u ch=%d fmt=%d period=%u",
              card, device,
              in->config.rate, in->config.channels, in->config.format,
              in->config.period_size);

        in->pcm = pcm_open(card, device, PCM_IN, &in->config);
        if(!pcm_is_ready(in->pcm)) {
            ALOGE("start_input_stream: cannot open pcm_in card=%d device=%d: %s",
                  card, device, pcm_get_error(in->pcm));
            pcm_close(in->pcm);
            in->pcm         = NULL;
            in->unavailable = true;
            return -ENODEV;
        }
        ALOGI("start_input_stream: opened HDMI PCM card=%d device=%d", card, device);
    } else {
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
    }

    adev->active_input = in;
    ALOGD("start_input_stream: done, active_input=%p", (void *)adev->active_input);
    return 0;
}

static int get_input_pcm_device(audio_devices_t device)
{
    if ((device & AUDIO_DEVICE_IN_HDMI) == AUDIO_DEVICE_IN_HDMI) {
        return 0;
    }
    return get_pcm_device();
}

static int probe_pcm_out_card() {
    FILE *fp;
    char card_node[32];
    char card_id[16];

    char card_prop[PROPERTY_VALUE_MAX];
    property_get("persist.audio.device", card_prop, "");

    for (int i = 0; i < 5; i++) {
        snprintf(card_node, sizeof(card_node), "/proc/asound/card%d/id", i);
        if ((fp = fopen(card_node, "r")) != NULL) {
            fgets(card_id, sizeof(card_id), fp);
            ALOGV("%s: %s", card_node, card_id);
            if (!strcmp(card_prop, "jack") && !strncmp(card_id, "Headphones", 10)) {
                fclose(fp);
                ALOGI("Using PCM card %d for 3.5mm audio jack", i);
                return i;
            } else if (!strcmp(card_prop, "dac") && strncmp(card_id, "Headphones", 10)
                    && strncmp(card_id, "vc4hdmi", 7)) {
                fclose(fp);
                ALOGI("Using PCM card %d for audio DAC %s", i, card_id);
                return i;
            }
            fclose(fp);
        }
    }

    ALOGE("Could not probe PCM card for %s, using PCM card 0", card_prop);
    return 0;
}

static int get_pcm_card()
{
    char card[PROPERTY_VALUE_MAX];
    property_get("persist.audio.pcm.card.auto", card, "false");
    if (!strcmp(card, "true"))
        return probe_pcm_out_card();

    property_get("persist.audio.pcm.card", card, "0");
    return atoi(card);
}

static int get_pcm_device()
{
    char device[PROPERTY_VALUE_MAX];
    property_get("persist.audio.pcm.device", device, "0");
    return atoi(device);
}

/* must be called with hw device and output stream mutexes locked */
static int start_output_stream(struct alsa_stream_out *out)
{
    struct alsa_audio_device *adev = out->dev;

    if (out->unavailable)
        return -ENODEV;

    /* default to low power: will be corrected in out_write if necessary before first write to
     * tinyalsa.
     */
    out->write_threshold = PLAYBACK_PERIOD_COUNT * PERIOD_SIZE;
    out->config.start_threshold = PLAYBACK_PERIOD_START_THRESHOLD * PERIOD_SIZE;
    out->config.avail_min = PERIOD_SIZE;

    int _out_card = pcm_card;
    int _out_dev  = pcm_device;
    ALOGD("start_output_stream: opening PCM card=%d device=%d "
          "rate=%u ch=%d fmt=%d period=%u count=%u thresh=%d",
          _out_card, _out_dev,
          out->config.rate, out->config.channels, out->config.format,
          out->config.period_size, out->config.period_count,
          out->config.start_threshold);

    out->pcm = pcm_open(_out_card, _out_dev, PCM_OUT | PCM_MMAP | PCM_NOIRQ | PCM_MONOTONIC, &out->config);

    if (!pcm_is_ready(out->pcm)) {
        ALOGE("start_output_stream: cannot open PCM card=%d device=%d: %s",
              _out_card, _out_dev, pcm_get_error(out->pcm));
        pcm_close(out->pcm);
        adev->active_output = NULL;
        out->unavailable = true;
        return -ENODEV;
    }

    adev->active_output = out;
    return 0;
}

static uint32_t out_get_sample_rate(const struct audio_stream *stream)
{
    struct alsa_stream_out *out = (struct alsa_stream_out *)stream;
    return out->config.rate;
}

static int out_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    ALOGV("out_set_sample_rate: %d", 0);
    return -ENOSYS;
}

static size_t out_get_buffer_size(const struct audio_stream *stream)
{
    ALOGV("out_get_buffer_size: %d", 4096);

    /* return the closest majoring multiple of 16 frames, as
     * audioflinger expects audio buffers to be a multiple of 16 frames */
    size_t size = PERIOD_SIZE;
    size = ((size + 15) / 16) * 16;
    return size * audio_stream_out_frame_size((struct audio_stream_out *)stream);
}

static audio_channel_mask_t out_get_channels(const struct audio_stream *stream)
{
    ALOGV("out_get_channels");
    struct alsa_stream_out *out = (struct alsa_stream_out *)stream;
    return audio_channel_out_mask_from_count(out->config.channels);
}

static audio_format_t out_get_format(const struct audio_stream *stream)
{
    ALOGV("out_get_format");
    struct alsa_stream_out *out = (struct alsa_stream_out *)stream;
    return audio_format_from_pcm_format(out->config.format);
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
        pcm_close(out->pcm);
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
    return (PERIOD_SIZE * PLAYBACK_PERIOD_COUNT * 1000) / out->config.rate;
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
    size_t out_frames = bytes / frame_size;

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

    ALOG_RATELIMIT(200, "out_write", "frames=%zu written_total=%u", out_frames, out->written);
    ret = pcm_mmap_write(out->pcm, buffer, out_frames * frame_size);
    if (ret == 0) {
        out->written += out_frames;
    } else {
        ALOGE("out_write: pcm_mmap_write failed: %s (frames=%zu)",
              pcm_get_error(out->pcm), out_frames);
    }
exit:
    pthread_mutex_unlock(&out->lock);

    if (ret != 0) {
        usleep((int64_t)bytes * 1000000 / audio_stream_out_frame_size(stream) /
                out_get_sample_rate(&stream->common));
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
        unsigned int avail;
        if (pcm_get_htimestamp(out->pcm, &avail, timestamp) == 0) {
            size_t kernel_buffer_size = out->config.period_size * out->config.period_count;
            int64_t signed_frames = out->written - kernel_buffer_size + avail;
            if (signed_frames >= 0) {
                *frames = signed_frames;
                ret = 0;
            }
        }
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

/** audio_stream_in implementation **/
static uint32_t in_get_sample_rate(const struct audio_stream *stream)
{
    struct alsa_stream_in *in = (struct alsa_stream_in *)stream;
    return in->config.rate ? in->config.rate : 48000;
}

static int in_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    ALOGV("in_set_sample_rate: %d", rate);
    return -ENOSYS;
}

static size_t in_get_buffer_size(const struct audio_stream *stream)
{
    struct alsa_stream_in *in = (struct alsa_stream_in *)stream;
    size_t period = in->config.period_size ? in->config.period_size : PERIOD_SIZE;
    size_t size = ((period + 15) / 16) * 16;
    return size * audio_stream_in_frame_size((struct audio_stream_in *)stream);
}

static audio_channel_mask_t in_get_channels(const struct audio_stream *stream)
{
    struct alsa_stream_in *in = (struct alsa_stream_in *)stream;
    return audio_channel_in_mask_from_count(in->config.channels);
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

    if(in->standby) {
        ret = start_input_stream(in);
        if(ret != 0) {
            pthread_mutex_unlock(&adev->lock);
            goto exit;
        }
        in->standby = 0;
    }
    pthread_mutex_unlock(&adev->lock);

    size_t in_frames = bytes / frame_size;

    // case HDIM
    if ((in->device & AUDIO_DEVICE_IN_HDMI) == AUDIO_DEVICE_IN_HDMI) {
        ret = pcm_read(in->pcm, buffer, in_frames * frame_size);
        if (ret != 0 && in->pcm)
            ALOGE("in_read: pcm_read failed: %s", pcm_get_error(in->pcm));

    } else {
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
            /* No FIFO data — (returns silence). */
            ALOG_RATELIMIT(300, "in_read", "HU mic starved (avail=%u < need=%zu), pcm_read fallback",
                        hu_avail, in_frames);
            ret = -1;
        }
        /* Rate-limit to real time: in_frames @ 48 kHz */
        usleep((long)in_frames * 1000000L / CODEC_SAMPLING_RATE);
    }

exit:
    pthread_mutex_unlock(&in->lock);

    if(ret != 0) {
        usleep((int64_t)bytes * 1000000 / frame_size /
                in_get_sample_rate(&stream->common));
        memset(buffer, 0, bytes);
    }
    return bytes;
}

static int do_input_standby(struct alsa_stream_in *in)
{
    if (!in->standby) {
        if (in->pcm) {
            pcm_close(in->pcm);
            in->pcm = NULL;
        }
        
        in->dev->active_input = NULL;
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
    struct pcm_params *params;
    int ret = 0;

    params = pcm_params_get(pcm_card, pcm_device, PCM_OUT);
    if (!params)
        return -ENOSYS;

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

    out->config.channels = CHANNEL_STEREO;
    out->config.rate = CODEC_SAMPLING_RATE;
    out->config.format = PCM_FORMAT_S16_LE;
    out->config.period_size = PERIOD_SIZE;
    out->config.period_count = PLAYBACK_PERIOD_COUNT;

    if (out->config.rate != config->sample_rate ||
           audio_channel_count_from_out_mask(config->channel_mask) != CHANNEL_STEREO ||
               out->config.format !=  pcm_format_from_audio_format(config->format) ) {
        config->sample_rate = out->config.rate;
        config->format = audio_format_from_pcm_format(out->config.format);
        config->channel_mask = audio_channel_out_mask_from_count(CHANNEL_STEREO);
        ret = -EINVAL;
    }

    ALOGI("adev_open_output_stream selects channels=%d rate=%d format=%d",
                out->config.channels, out->config.rate, out->config.format);

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

static size_t adev_get_input_buffer_size(const struct audio_hw_device *dev,
        const struct audio_config *config)
{
    size_t frame_size = audio_bytes_per_sample(config->format) *
                        popcount(config->channel_mask);
    size_t frames = PERIOD_SIZE;
    size_t buf_size = frames * frame_size;
    ALOGV("adev_get_input_buffer_size: %zu", buf_size);
    return buf_size;
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

    in = (struct alsa_stream_in *)calloc(1, sizeof(*in));
    if (!in)
        return -ENOMEM;

    ALOGV("adev_open_input_stream...");

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

    in->device = devices;
    in->dev = ladev;
    in->standby = 1;
    in->config = in_config;

    ALOGI("adev_open_input_stream: device=0x%08x rate=%u ch_mask=0x%x fmt=%d flags=0x%x",
          devices, config->sample_rate, config->channel_mask, config->format, flags);

    if ((in->device & AUDIO_DEVICE_IN_HDMI) == AUDIO_DEVICE_IN_HDMI) {
        in->config.period_size = 256;
        in->config.start_threshold = 256*8;
    }

    config->sample_rate = 48000;
    config->channel_mask = AUDIO_CHANNEL_IN_STEREO;
    config->format = AUDIO_FORMAT_PCM_16_BIT;

    *stream_in = &in->stream;
    return 0;
}

static void adev_close_input_stream(struct audio_hw_device *dev,
        struct audio_stream_in *stream)
{
    struct alsa_stream_in *in = (struct alsa_stream_in *)stream;
    int status = do_input_standby(in);
    free(in);
}

static int adev_dump(const audio_hw_device_t *device, int fd)
{
    ALOGV("adev_dump");
    return 0;
}

static int adev_close(hw_device_t *device)
{
    ALOGV("adev_close");
    struct alsa_audio_device *adev = (struct alsa_audio_device *)device;
    if (adev->hu_mic_fd >= 0) {
        close(adev->hu_mic_fd);
        adev->hu_mic_fd = -1;
    }
    free(device);
    return 0;
}

static int adev_open(const hw_module_t* module, const char* name,
        hw_device_t** device)
{
    struct alsa_audio_device *adev;

    ALOGV("adev_open: %s", name);

    pcm_card = get_pcm_card();
    pcm_device = get_pcm_device();
    ALOGI("adev_open: pcm_card %d, pcm_device %d", pcm_card, pcm_device);

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

    /* Open read end of the HU mic FIFO non-blocking.
     * The FIFO is created by init.rc at boot (as root). */
    adev->hu_mic_fd = open(HU_MIC_FIFO_PATH, O_RDONLY | O_NONBLOCK);
    adev->hu_mic_wr = 0;
    adev->hu_mic_rd = 0;
    ALOGI("hu_mic: FIFO %s opened fd=%d", HU_MIC_FIFO_PATH, adev->hu_mic_fd);

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
        .name = "Raspberry Pi audio HW HAL",
        .author = "The Android Open Source Project",
        .methods = &hal_module_methods,
    },
};
