#!/bin/bash
# ラズパイから（SSH でも可）Pico に .uf2 を書き込む。BOOTSEL ボタンを押す必要なし。
# 使い方:  ./flash_pico.sh robot_pico.uf2 [/dev/ttyACM0]
set -e
UF2=${1:?"書き込む .uf2 ファイルを指定してください"}
PORT=${2:-/dev/ttyACM0}

find_drive() {
  lsblk -rno NAME,LABEL | awk '$2=="RP2350" || $2=="RPI-RP2" {print "/dev/"$1}' | head -1
}
wait_drive() {
  for _ in $(seq 1 20); do DEV=$(find_drive); [ -n "$DEV" ] && return 0; sleep 0.5; done
  return 1
}

DEV=$(find_drive)
if [ -z "$DEV" ] && [ -e "$PORT" ]; then
  echo "Pico を書き込みモードにします（1200bps の合図）..."
  stty -F "$PORT" 1200 2>/dev/null || true
  if ! wait_drive; then
    echo "MicroPython の場合の方法で書き込みモードにします..."
    stty -F "$PORT" 115200 raw -echo 2>/dev/null || true
    printf '\x03\x03\r\nimport machine; machine.bootloader()\r\n' > "$PORT" || true
    wait_drive || true
  fi
fi
DEV=$(find_drive)
if [ -z "$DEV" ]; then
  echo "書き込みモードの Pico が見つかりません。BOOTSEL ボタンを押しながら USB を挿し直して、もう一度実行してください。"
  exit 1
fi
MNT=$(mktemp -d)
sudo mount "$DEV" "$MNT"
echo "書き込み中: $UF2 → $DEV"
sudo cp "$UF2" "$MNT/"
sync
sudo umount "$MNT" 2>/dev/null || true
rmdir "$MNT" 2>/dev/null || true
echo "完了。Pico は自動で再起動します。"
