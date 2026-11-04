"""Pico（C 版・MicroPython 版どちらも）に手でコマンドを送って確かめるツール。ラズパイでも Windows でも動く。
使い方:  python3 pico_term.py [ポート]      例) /dev/ttyACM0、COM5
  ?               設定と状態
  T 2             状態を 1 秒に 2 回表示（T 0 で止める）
  M 0.5 0.5       左右を強さ 50% で回す（0.5 秒で自動停止）
  W 1000 1000     左右を 1000 カウント/秒で回す（0.5 秒で自動停止）
  r 3 W 1000 1000 「W 1000 1000」を 3 秒間くり返し送る（続けて回したいとき）
  S               止める
終了は Ctrl+C（止めてから終わります）
"""
import sys
import threading
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
ser = serial.Serial(port, 115200, timeout=0.1)


def reader():
    while True:
        try:
            line = ser.readline()
        except serial.SerialException:
            print('通信が切れました')
            return
        if line:
            print('<', line.decode(errors='ignore').rstrip())


threading.Thread(target=reader, daemon=True).start()
print(__doc__.split('\n', 2)[2])
try:
    while True:
        cmd = input().strip()
        if not cmd:
            continue
        p = cmd.split(None, 2)
        if p[0] == 'r' and len(p) == 3:
            end = time.time() + float(p[1])
            while time.time() < end:
                ser.write((p[2] + '\n').encode())
                time.sleep(0.1)
            ser.write(b'S\n')
        else:
            ser.write((cmd + '\n').encode())
except (KeyboardInterrupt, EOFError):
    pass
finally:
    ser.write(b'S\nT 0\n')
    time.sleep(0.1)
