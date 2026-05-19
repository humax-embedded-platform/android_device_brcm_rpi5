#!/bin/bash
# Push audio HAL .so files to Head Unit via ADB

set -e

DEVICE_LIB_PATH="/vendor/lib/hw"
DEVICE_ETC_PATH="/vendor/etc"
AOSP_OUT="${ANDROID_PRODUCT_OUT:-}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

LIBS=(
    "audio.primary.rpi.so"
    "audio.primary.rpi_hdmi.so"
)

XML_SRC="$SCRIPT_DIR/audio_policy_configuration.xml"
XML_DST="$DEVICE_ETC_PATH/audio_policy_configuration.xml"

# --- helpers ---

die() { echo "ERROR: $*" >&2; exit 1; }

find_so() {
    local name="$1"
    if [[ -n "$AOSP_OUT" ]]; then
        local p="$AOSP_OUT/vendor/lib/hw/$name"
        [[ -f "$p" ]] && { echo "$p"; return; }
    fi
    local root
    root="$(cd "$SCRIPT_DIR/../../../.." && pwd)"   # device/brcm/rpi4/audio -> root
    local found
    found="$(find "$root/out" -name "$name" -path "*/vendor/lib/hw/*" 2>/dev/null | head -n1)"
    [[ -n "$found" ]] && { echo "$found"; return; }
    die "Cannot find $name. Set ANDROID_PRODUCT_OUT or run from AOSP root after a build."
}

push_xml() {
    [[ -f "$XML_SRC" ]] || die "XML not found: $XML_SRC"
    echo "Pushing audio_policy_configuration.xml  ->  $XML_DST"
    adb push "$XML_SRC" "$XML_DST"
    adb shell chmod 644 "$XML_DST"
}

check_adb() {
    command -v adb >/dev/null 2>&1 || die "'adb' not found in PATH."
    adb wait-for-device
    local state
    state="$(adb get-state 2>/dev/null)"
    [[ "$state" == "device" ]] || die "Device not ready (state: $state)."
}

remount_vendor() {
    echo "Remounting /vendor rw..."
    adb root >/dev/null
    adb wait-for-device
    adb remount >/dev/null || true
    # Fallback for devices without overlayfs
    adb shell mount -o remount,rw /vendor 2>/dev/null || true
}

push_lib() {
    local src="$1" name
    name="$(basename "$src")"
    echo "Pushing $name  ->  $DEVICE_LIB_PATH/$name"
    adb push "$src" "$DEVICE_LIB_PATH/$name"
    adb shell chmod 644 "$DEVICE_LIB_PATH/$name"
}

# --- parse args ---

REBOOT=false
PUSH_XML=false
SELECT=()

usage() {
    echo "Usage: $0 [OPTIONS] [rpi|rpi_hdmi|xml|all]"
    echo ""
    echo "  all        Push both .so libs + xml (default)"
    echo "  rpi        Push audio.primary.rpi.so only"
    echo "  rpi_hdmi   Push audio.primary.rpi_hdmi.so only"
    echo "  xml        Push audio_policy_configuration.xml only"
    echo ""
    echo "Options:"
    echo "  -r, --reboot   Reboot device after push"
    echo "  -s <serial>    Target specific ADB serial"
    echo "  -h, --help     Show this help"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -r|--reboot) REBOOT=true ;;
        -s) export ANDROID_SERIAL="$2"; shift ;;
        -h|--help) usage; exit 0 ;;
        all)      SELECT=("${LIBS[@]}"); PUSH_XML=true ;;
        rpi)      SELECT=("audio.primary.rpi.so") ;;
        rpi_hdmi) SELECT=("audio.primary.rpi_hdmi.so") ;;
        xml)      PUSH_XML=true ;;
        *) die "Unknown argument: $1. Use -h for help." ;;
    esac
    shift
done

# Default: push everything
if [[ ${#SELECT[@]} -eq 0 ]] && ! $PUSH_XML; then
    SELECT=("${LIBS[@]}")
    PUSH_XML=true
fi

# --- main ---

check_adb
remount_vendor

for lib in "${SELECT[@]}"; do
    src="$(find_so "$lib")"
    push_lib "$src"
done

$PUSH_XML && push_xml

if $REBOOT; then
    echo "Rebooting device..."
    adb reboot
else
    echo "Restarting audioserver..."
    adb shell stop audioserver && adb shell start audioserver
fi

echo "Done."
