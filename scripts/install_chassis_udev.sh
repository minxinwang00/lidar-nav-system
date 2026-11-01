#!/usr/bin/env bash
# Install a stable /dev/minipc symlink for the STM32 chassis CDC.
# Run once with: sudo bash ~/nav_system_ws/scripts/install_chassis_udev.sh
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "请用 sudo 运行：sudo bash $0"
  exit 1
fi

SERIAL="${1:-206532874D31}"
RULE=/etc/udev/rules.d/99-minipc.rules

cat > "${RULE}" <<EOF
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{serial}=="${SERIAL}", ATTRS{bInterfaceNumber}=="00", MODE="0666", GROUP="dialout", SYMLINK+="minipc"
EOF

echo "已写入 ${RULE}:"
cat "${RULE}"

udevadm control --reload-rules
udevadm trigger --subsystem-match=tty

for _ in $(seq 1 25); do
  if [[ -e /dev/minipc ]]; then
    break
  fi
  sleep 0.2
done

if [[ -e /dev/minipc ]]; then
  echo "成功："
  ls -l /dev/minipc
  readlink -f /dev/minipc
else
  echo "规则已安装，但 /dev/minipc 还没出现。"
  echo "请拔掉底盘 USB 再插回，然后执行：ls -l /dev/minipc"
fi
