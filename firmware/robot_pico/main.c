// ロボット足回り Pico 2 ファームウェア（C / Pico SDK）
// MicroPython 版 main.py と同じ通信の決まりなので、ラズパイ側の pico_bridge はそのまま使える。
//
// ラズパイ → Pico（USB シリアル、1行1コマンド）
//   W <l> <r>   左右のタイヤの速さ [カウント/秒] で速度制御（pico_bridge が使う）
//   V <v> <w>   進む速さ [m/s]・回る速さ [rad/s]（左回りプラス）で速度制御（手動テスト用）
//   M <l> <r>   左右の強さ -1.0〜1.0 で直接回す（テスト用）
//   S           止める（ブレーキ）
//   T <hz>      状態の送信回数 [回/秒]（0 で送らない。起動直後は 0）
//   Z           エンコーダのカウントを 0 に戻す
//   ?           設定と状態を1行で返す
//   W・V・M は 0.5 秒指令が来ないと自動で止まる。
// Pico → ラズパイ
//   D <ms> <左カウント> <右カウント> <gx> <gy> <gz> <ax> <ay> <az>
//   （ms は 2^30 で一周。カウントは前進でプラス。g は rad/s、a は m/s^2。IMU が読めないときは nan）
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "pico/stdio_usb.h"

// ---------------- 寸法（V コマンドの手動テストにだけ使う。ROS 2 側は pico_bridge.yaml） ----------------
#define WHEEL_DIAMETER 0.065f
#define TRACK_WIDTH    0.20f
#define TICKS_PER_REV  1000.0f

// ---------------- ピン（接続チェックで確定） ----------------
#define L_FWD 10
#define L_REV 11
#define R_FWD 13
#define R_REV 12
#define L_C1 5
#define L_C2 4
#define R_C1 20
#define R_C2 21
static const int SIGN[2] = {-1, 1};      // 前進でプラスになるよう左は反転
#define IMU_SDA 2
#define IMU_SCL 3
#define IMU_ADDR 0x6B
#define IMU_I2C i2c1

// ---------------- 制御 ----------------
#define PWM_FREQ 20000u
#define LOOP_MS 20
#define CMD_TIMEOUT_MS 500
static const float KF = 1.0f / 4300.0f, KP = 0.0003f, KI = 0.002f;

// ================= モータ（スローディケイ） =================
static uint16_t pwm_top;
static const uint MOTOR_PIN[2][2] = {{L_FWD, L_REV}, {R_FWD, R_REV}};

static void pwm_setup(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    uint32_t sys = clock_get_hz(clk_sys);
    uint32_t top = sys / PWM_FREQ - 1;          // 150MHz なら 7499
    pwm_top = (uint16_t)top;
    pwm_set_wrap(slice, pwm_top);
    pwm_set_clkdiv(slice, 1.0f);
    pwm_set_gpio_level(pin, 0);
    pwm_set_enabled(slice, true);
}

static void set_level(uint pin, float frac) {   // frac 0..1 = HIGH の割合
    if (frac < 0) frac = 0;
    if (frac > 1) frac = 1;
    pwm_set_gpio_level(pin, (uint16_t)(frac * (pwm_top + 1)));
}

static void drive(int k, float v) {             // v -1..1、0 はブレーキ
    if (v > 1) v = 1;
    if (v < -1) v = -1;
    uint a = MOTOR_PIN[k][0], b = MOTOR_PIN[k][1];
    if (v > 0) {
        set_level(a, 1); set_level(b, 1 - v);
    } else if (v < 0) {
        set_level(b, 1); set_level(a, 1 + v);
    } else {
        set_level(a, 1); set_level(b, 1);
    }
}

static void brake_all(void) { drive(0, 0); drive(1, 0); }

// ================= エンコーダ（GPIO 割り込みで 4 逓倍。MicroPython 版と同じ表） =================
static const int8_t TAB[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
static const uint ENC_PIN[2][2] = {{L_C1, L_C2}, {R_C1, R_C2}};
static volatile int32_t enc_pos[2];
static volatile uint8_t enc_state[2];

static inline uint8_t enc_read(int k) {
    return (uint8_t)((gpio_get(ENC_PIN[k][0]) << 1) | gpio_get(ENC_PIN[k][1]));
}

static void gpio_cb(uint gpio, uint32_t events) {
    (void)events;
    for (int k = 0; k < 2; k++) {
        if (gpio == ENC_PIN[k][0] || gpio == ENC_PIN[k][1]) {
            uint8_t s = enc_read(k);
            enc_pos[k] += TAB[(enc_state[k] << 2) | s];
            enc_state[k] = s;
        }
    }
}

static void enc_setup(void) {
    for (int k = 0; k < 2; k++) {
        for (int c = 0; c < 2; c++) {
            uint p = ENC_PIN[k][c];
            gpio_init(p);
            gpio_set_dir(p, GPIO_IN);
            gpio_pull_up(p);
        }
    }
    sleep_us(100);
    for (int k = 0; k < 2; k++) enc_state[k] = enc_read(k);
    gpio_set_irq_enabled_with_callback(ENC_PIN[0][0], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_cb);
    gpio_set_irq_enabled(ENC_PIN[0][1], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_PIN[1][0], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_PIN[1][1], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

static void enc_get(int32_t out[2]) {
    uint32_t irq = save_and_disable_interrupts();
    out[0] = enc_pos[0] * SIGN[0];
    out[1] = enc_pos[1] * SIGN[1];
    restore_interrupts(irq);
}

static void enc_zero(void) {
    uint32_t irq = save_and_disable_interrupts();
    enc_pos[0] = enc_pos[1] = 0;
    restore_interrupts(irq);
}

// ================= IMU（LSM6DSV16X） =================
static bool imu_ok = false;
static const float G_SCALE = 0.0175f * 3.14159265f / 180.0f;   // ±500 dps
static const float A_SCALE = 0.000122f * 9.80665f;             // ±4 g
#define I2C_TO_US 2000

static void pin_in(uint p, bool up) {
    gpio_set_function(p, GPIO_FUNC_SIO);
    gpio_set_dir(p, GPIO_IN);
    if (up) gpio_pull_up(p); else gpio_pull_down(p);
}
static void pin_out(uint p, bool v) {
    gpio_set_function(p, GPIO_FUNC_SIO);
    gpio_put(p, v);
    gpio_set_dir(p, GPIO_OUT);
}

static void imu_recover(void) {      // SDA・SCL を直接動かして「読めない状態」から戻す（実機で効いた手順）
    i2c_deinit(IMU_I2C);
    pin_in(IMU_SCL, true);  pin_out(IMU_SDA, 0); sleep_ms(5);
    pin_in(IMU_SCL, false); gpio_put(IMU_SDA, 1); sleep_ms(5);
    pin_in(IMU_SDA, true);  pin_out(IMU_SCL, 0); sleep_ms(5);
    pin_in(IMU_SDA, false); gpio_put(IMU_SCL, 1); sleep_ms(5);
    pin_in(IMU_SCL, true);  pin_in(IMU_SDA, true); sleep_ms(5);
}

static void i2c_start(void) {
    i2c_init(IMU_I2C, 400 * 1000);
    gpio_set_function(IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA);
    gpio_pull_up(IMU_SCL);
}

static bool imu_wr(uint8_t reg, uint8_t val) {
    uint8_t b[2] = {reg, val};
    return i2c_write_timeout_us(IMU_I2C, IMU_ADDR, b, 2, false, I2C_TO_US) == 2;
}

static bool imu_rd(uint8_t reg, uint8_t *buf, size_t n) {
    if (i2c_write_timeout_us(IMU_I2C, IMU_ADDR, &reg, 1, true, I2C_TO_US) != 1) return false;
    return i2c_read_timeout_us(IMU_I2C, IMU_ADDR, buf, n, false, I2C_TO_US) == (int)n;
}

static bool imu_try_init(void) {
    i2c_start();
    uint8_t d;
    i2c_read_timeout_us(IMU_I2C, IMU_ADDR, &d, 1, false, I2C_TO_US);   // 起こしの1バイト読み
    if (!imu_rd(0x0F, &d, 1) || d != 0x70) return false;              // WHO_AM_I
    return imu_wr(0x10, 0x06)       // CTRL1：加速度 120Hz
        && imu_wr(0x11, 0x06)       // CTRL2：ジャイロ 120Hz
        && imu_wr(0x15, 0x02)       // CTRL6：ジャイロ ±500 dps
        && imu_wr(0x17, 0x01);      // CTRL8：加速度 ±4 g
}

static bool imu_init(void) {
    imu_ok = imu_try_init();
    for (int k = 0; k < 3 && !imu_ok; k++) {
        imu_recover();
        imu_ok = imu_try_init();
    }
    return imu_ok;
}

static bool imu_read(float g[6]) {
    uint8_t b[12];
    if (!imu_rd(0x22, b, 12)) return false;
    for (int i = 0; i < 6; i++) {
        int16_t v = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));
        g[i] = v * (i < 3 ? G_SCALE : A_SCALE);
    }
    return true;
}

// ================= コマンド =================
typedef enum { MODE_STOP, MODE_VEL, MODE_RAW } drive_mode_t;
static drive_mode_t mode = MODE_STOP;
static float target[2];
static uint32_t last_cmd_ms;
static int tele_every = 0;
static char line[96];
static int line_len = 0;

static float mps_to_tps(float v) { return v / (3.14159265f * WHEEL_DIAMETER) * TICKS_PER_REV; }

static void handle(char *s) {
    char *p = s;
    while (*p == ' ') p++;
    char c = (char)toupper((unsigned char)*p);
    if (!c) return;
    char *rest = p + 1;
    float a = 0, b = 0;
    int n = sscanf(rest, "%f %f", &a, &b);
    uint32_t now = to_ms_since_boot(get_absolute_time());
    switch (c) {
    case 'W':
        if (n != 2) goto bad;
        target[0] = a; target[1] = b; mode = MODE_VEL; last_cmd_ms = now; break;
    case 'V': {
        if (n != 2) goto bad;
        float half = b * TRACK_WIDTH / 2;
        target[0] = mps_to_tps(a - half); target[1] = mps_to_tps(a + half);
        mode = MODE_VEL; last_cmd_ms = now; break;
    }
    case 'M':
        if (n != 2) goto bad;
        target[0] = a; target[1] = b; mode = MODE_RAW; last_cmd_ms = now; break;
    case 'S':
        mode = MODE_STOP; break;
    case 'T':
        if (n < 1) goto bad;
        tele_every = (a <= 0) ? 0 : (int)lroundf(1000.0f / LOOP_MS / a);
        if (a > 0 && tele_every < 1) tele_every = 1;
        break;
    case 'Z':
        enc_zero(); break;
    case '?': {
        int32_t t[2];
        enc_get(t);
        printf("I fw=c wheel_d=%.3f track=%.3f ticks_per_rev=%.0f imu=%d mode=%d ticks=%ld %ld\n",
               WHEEL_DIAMETER, TRACK_WIDTH, TICKS_PER_REV, imu_ok, (int)mode, (long)t[0], (long)t[1]);
        break;
    }
    default:
        printf("E unknown command: %s\n", s);
    }
    return;
bad:
    printf("E bad command: %s\n", s);
}

static void read_cmds(void) {
    for (;;) {
        int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT) return;
        if (ch == '\n' || ch == '\r') {
            if (line_len) { line[line_len] = 0; handle(line); }
            line_len = 0;
        } else if (line_len < (int)sizeof(line) - 1) {
            line[line_len++] = (char)ch;
        } else {
            line_len = 0;
        }
    }
}

// ================= メイン =================
int main(void) {
    for (int k = 0; k < 2; k++)
        for (int c = 0; c < 2; c++) pwm_setup(MOTOR_PIN[k][c]);
    brake_all();
    stdio_init_all();
    enc_setup();
    imu_init();
    watchdog_enable(1000, true);           // 1 秒固まったら再起動（モータも止まる）

    int32_t prev[2], cur[2];
    float meas[2] = {0, 0}, integ[2] = {0, 0};
    enc_get(prev);
    int imu_err = 0;
    uint32_t n = 0;
    bool said_ready = false;
    absolute_time_t t_next = make_timeout_time_ms(LOOP_MS);

    for (;;) {
        watchdog_update();
        read_cmds();
        if (!said_ready && stdio_usb_connected()) {
            printf("I ready fw=c imu=%d\n", imu_ok);
            said_ready = true;
        }
        if (absolute_time_diff_us(get_absolute_time(), t_next) > 0) {
            sleep_us(200);
            continue;
        }
        t_next = delayed_by_ms(t_next, LOOP_MS);
        if (absolute_time_diff_us(get_absolute_time(), t_next) < 0) t_next = make_timeout_time_ms(LOOP_MS);

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (mode != MODE_STOP && (uint32_t)(now - last_cmd_ms) > CMD_TIMEOUT_MS) mode = MODE_STOP;

        const float dt = LOOP_MS / 1000.0f;
        enc_get(cur);
        for (int k = 0; k < 2; k++) {
            meas[k] = 0.5f * meas[k] + 0.5f * (float)(cur[k] - prev[k]) / dt;
            prev[k] = cur[k];
        }

        if (mode == MODE_VEL) {
            for (int k = 0; k < 2; k++) {
                if (target[k] == 0) { integ[k] = 0; drive(k, 0); continue; }
                float err = target[k] - meas[k];
                float u = KF * target[k] + KP * err + KI * integ[k];
                if (u > -1 && u < 1) integ[k] += err * dt;
                drive(k, u);
            }
        } else if (mode == MODE_RAW) {
            drive(0, target[0]);
            drive(1, target[1]);
        } else {
            integ[0] = integ[1] = 0;
            brake_all();
        }

        n++;
        if (tele_every && (n % (uint32_t)tele_every) == 0) {
            float g[6];
            bool got = imu_ok && imu_read(g);
            if (imu_ok) imu_err = got ? 0 : imu_err + 1;
            uint32_t ms30 = now & ((1u << 30) - 1);
            if (got) {
                printf("D %lu %ld %ld %.4f %.4f %.4f %.3f %.3f %.3f\n", (unsigned long)ms30, (long)cur[0], (long)cur[1],
                       g[0], g[1], g[2], g[3], g[4], g[5]);
            } else {
                printf("D %lu %ld %ld nan nan nan nan nan nan\n", (unsigned long)ms30, (long)cur[0], (long)cur[1]);
                if (imu_err >= 25) { brake_all(); imu_init(); imu_err = 0; }
            }
        }
    }
}
