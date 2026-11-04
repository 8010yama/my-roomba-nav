# robot_pico（足回り Pico 2 の C 版ファームウェア）

MicroPython 版 `main.py` と同じ動きを C（Pico SDK）で書いたものです。通信の決まりが同じなので、ラズパイ側の `pico_bridge` はそのまま使えます。

| ファイル | 中身 |
|---|---|
| `robot_pico.uf2` | ビルド済みのファームウェア（Pico 2 / RP2350 用）。リポジトリには入れていないので、下の「作り直すとき」の手順でビルドして作る |
| `flash_pico.sh` | ラズパイから（SSH でも）書き込むスクリプト |
| `pico_term.py` | 手でコマンドを送って動きを確かめるツール |
| `main.c`・`CMakeLists.txt` | ソース（作り直すとき用） |

## 書き込み方

### A. ラズパイから（SSH でつないだままで OK、ボタン操作なし）

```bash
chmod +x flash_pico.sh
./flash_pico.sh robot_pico.uf2
```

Pico を自動で書き込みモードにして、書き込み、再起動まで行います。今 MicroPython が入っている Pico にも使えます。2回目以降（C 版から C 版への更新）も同じコマンドです。

うまくいかないときだけ、Pico の BOOTSEL ボタンを押しながら USB を挿し直してから、もう一度実行してください。

### B. パソコンから

BOOTSEL ボタンを押しながら USB を挿すと「RP2350」というドライブが出るので、`robot_pico.uf2` をドラッグして入れます。

### MicroPython に戻したいとき

同じ手順で、MicroPython の .uf2（Pico 2 用）を書き込みます。

## 動きの確かめ方

C 版は Thonny のシェルからは操作できないので、`pico_term.py` を使います（`pip install pyserial` が必要）。

```bash
python3 pico_term.py /dev/ttyACM0      # Windows なら COM5 など
```

タイヤを浮かせて、電池をつないでから：

- `?` → `I fw=c ... imu=1` と出れば OK（imu=1 は IMU が読めている印）
- `T 2` → 1 秒に 2 回、カウントと IMU の値が出る（`T 0` で止める）
- `r 2 M 0.5 0.5` → 2 秒間、左右とも前に回る
- `r 2 W 1500 1500` → 2 秒間、1500 カウント/秒で回る（速度制御）

確かめ終わったら `pico_term.py` を閉じてから、`ros2 launch pico_bridge bringup.launch.py` を起動します（同じポートは同時に1つしか開けません）。

## MicroPython 版との違い

- エンコーダの割り込みが C なので、速く回してもカウントを取りこぼしにくい
- 1 秒以上プログラムが止まると自動で再起動する（ウォッチドッグ。モータも止まる）
- `?` の答えに `fw=c` が付く

## 作り直すとき（ソースを変えたら）

Pico SDK 2.1.1 と arm-none-eabi-gcc が必要です。

```bash
export PICO_SDK_PATH=~/pico-sdk
mkdir build && cd build
cmake -G Ninja .. && ninja      # → build/robot_pico.uf2
```
