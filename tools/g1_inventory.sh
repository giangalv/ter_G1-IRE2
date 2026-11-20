#!/bin/bash
# READ-ONLY inventory of the robot's Jetson (PC2): module, JetPack, CPU, memory, storage, USB devices, network, audio, cameras,
# Unitree modules. Nothing is written or changed on the robot (no sudo, no service touched). Feeds docs/09-datasheet.md.
#
#   G1_PW=<Jetson password> tools/g1_inventory.sh > inventory.txt     # cable, or G1_JETSON_IP=10.42.0.1 on the robot's Wi-Fi
#
# MAC addresses and serial-number-looking strings are masked in the output (they must not end up in the repositories).
set -u
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
JETSON_IP=${G1_JETSON_IP:-192.168.123.164}
if [ -n "${G1_PW:-}" ]; then
    export SSH_ASKPASS="$HERE/askpass.sh" SSH_ASKPASS_REQUIRE=force DISPLAY=${DISPLAY:-:0}
fi
JETSON_SSH=${G1_JETSON_SSH:-ssh -o StrictHostKeyChecking=accept-new -o PreferredAuthentications=password -o PubkeyAuthentication=no -o ConnectTimeout=8 unitree@$JETSON_IP}

remote_script() {
    cat <<'EOF'
sec() { printf '\n## %s\n' "$1"; }
sec "system"
echo "model: $(tr -d '\0' < /proc/device-tree/model 2>/dev/null)"
echo "os: $(lsb_release -ds 2>/dev/null) | kernel $(uname -r) | $(uname -m)"
echo "l4t: $(head -1 /etc/nv_tegra_release 2>/dev/null | cut -c1-120)"
echo "jetpack: $(dpkg-query -W -f='${Version}' nvidia-jetpack 2>/dev/null)"
echo "uptime: $(uptime -p)"
sec "cpu"
lscpu 2>/dev/null | grep -E "^(Model name|CPU\(s\)|Thread|Core|Socket|CPU max MHz|Architecture)"
echo "online cpus: $(cat /sys/devices/system/cpu/online)"
sec "memory"
grep -E "MemTotal|SwapTotal" /proc/meminfo
sec "gpu/power (tegrastats, 2 s)"
timeout 3 tegrastats --interval 1000 2>/dev/null | head -2
echo "nvpmodel -q: $(nvpmodel -q 2>&1 | tr '\n' ' ' | cut -c1-160)"
grep -E "PM_CONFIG DEFAULT|^< POWER_MODEL ID" /etc/nvpmodel.conf 2>/dev/null | head -8
sec "thermal (deg C)"
for z in /sys/class/thermal/thermal_zone*; do echo "$(cat $z/type): $(( $(cat $z/temp) / 1000 ))"; done 2>/dev/null
sec "storage"
lsblk -d -o NAME,SIZE,MODEL,TRAN 2>/dev/null
df -h / /home 2>/dev/null | awk 'NR==1 || !seen[$1 $6]++'
sec "usb"
lsusb 2>/dev/null
sec "usb tree"
lsusb -t 2>/dev/null | head -30
sec "network"
ip -br addr 2>/dev/null
for n in /sys/class/net/e*; do echo "$(basename $n) speed: $(cat $n/speed 2>/dev/null) Mb/s"; done 2>/dev/null
iw dev 2>/dev/null | grep -E "Interface|type|channel|ssid"
sec "bluetooth"
rfkill list 2>/dev/null | head -8
hciconfig -a 2>/dev/null | grep -E "^hci|Name|HCI Version|LMP Version" | head -6
sec "audio"
aplay -l 2>/dev/null | grep -E "^card"
arecord -l 2>/dev/null | grep -E "^card"
sec "video nodes (by-path)"
ls /dev/v4l/by-path 2>/dev/null | sed 's/^platform-//' | head -20
sec "software"
echo "ros: $(ls /opt/ros 2>/dev/null | tr '\n' ' ') | python $(python3 -V 2>&1) | docker: $(docker --version 2>&1 | cut -c1-60)"
echo "unitree: $(ls /unitree 2>/dev/null | tr '\n' ' ')"
echo "modules: $(ls /unitree/module 2>/dev/null | tr '\n' ' ')"
EOF
}

remote_script | $JETSON_SSH 'bash -s' 2>&1 \
    | sed -E 's/([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}/xx:xx:xx:xx:xx:xx/g'
