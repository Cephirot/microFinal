/*
 * Beatbox Maker — nRF52840 DK + Open Smart Rich Shield TWO
 *
 * QDEC 인코더  → 볼륨
 * 조이스틱 좌우 → 볼륨 (보조)
 * 조이스틱 상하 → SEL:UP / SEL:DOWN  (사운드 탐색)
 * nRF 버튼 1~4  → SLOT:1 ~ SLOT:4    (슬롯 배정)
 */

#include <bluetooth/services/nus.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(beatbox, LOG_LEVEL_INF);

/* ── LED ─────────────────────────────────── */
#define LED_COUNT 4
static const struct gpio_dt_spec leds[LED_COUNT] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios),
};

/* ── nRF DK 버튼 ─────────────────────────── */
static const struct gpio_dt_spec btns[4] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios),
};

/* ── 조이스틱 ADC ────────────────────────── */
/* X축 A0=P0.03=AIN1: 좌우 (LEFT < 중립 < RIGHT) */
/* Y축 A1=P0.04=AIN2: 상하 (DOWN < 중립 < UP)    */
#define JOY_X_LEFT_MAX  300   /* X 측정값 ≈ 2 */
#define JOY_X_RIGHT_MIN 2700  /* X 측정값 ≈ 3026 */
#define JOY_X_RIGHT_MAX 3100
#define JOY_Y_UP_MIN    2700  /* UP: 2700~3200 (측정값 ≈ 3017) */
#define JOY_Y_UP_MAX    3200
#define JOY_Y_DOWN_MAX   300  /* DOWN: < 300 (측정값 ≈ -1~4) */

#define VOL_STEP    2
#define POLL_MS     150

static int enc_vol = 50;
static const struct device *qdec_dev;
static const struct device *adc_dev;

/* X축 (좌우) */
static int16_t joy_x_buf;
static struct adc_sequence joy_x_seq;
static const struct adc_channel_cfg joy_x_cfg = {
    .gain             = ADC_GAIN_1_4,
    .reference        = ADC_REF_VDD_1_4,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = 1,
    .input_positive   = SAADC_CH_PSELP_PSELP_AnalogInput1,
};

/* Y축 (상하) */
static int16_t joy_y_buf;
static struct adc_sequence joy_y_seq;
static const struct adc_channel_cfg joy_y_cfg = {
    .gain             = ADC_GAIN_1_4,
    .reference        = ADC_REF_VDD_1_4,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = 2,
    .input_positive   = SAADC_CH_PSELP_PSELP_AnalogInput2,
};

/* ── BLE ─────────────────────────────────── */
static struct bt_conn *current_conn;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, "BeatboxMaker", sizeof("BeatboxMaker") - 1),
};

static void send_ble(const char *msg) {
    if (current_conn)
        bt_nus_send(current_conn, (uint8_t *)msg, strlen(msg));
}

static void led_set(int idx, bool on) {
    if (idx < 0 || idx >= LED_COUNT) return;
    gpio_pin_set_dt(&leds[idx], on ? 1 : 0);
    char msg[16];
    snprintf(msg, sizeof(msg), "LED:%d:%d", idx + 1, on ? 1 : 0);
    send_ble(msg);
}

/* ── BLE 콜백 ────────────────────────────── */
static void on_nus_received(struct bt_conn *conn, const uint8_t *const data,
                            uint16_t len) {
    char cmd[64] = {0};
    memcpy(cmd, data, MIN(len, sizeof(cmd) - 1));
    LOG_INF("RX: %s", cmd);

    if (strncmp(cmd, "PLAY:", 5) == 0) {
        int slot = atoi(&cmd[5]);
        if (slot >= 1 && slot <= 4) {
            led_set(slot - 1, true);
        }
    } else if (strncmp(cmd, "STOP:", 5) == 0) {
        int slot = atoi(&cmd[5]);
        if (slot >= 1 && slot <= 4) {
            led_set(slot - 1, false);
        }
    }
}

static void on_connected(struct bt_conn *conn, uint8_t err) {
    if (err) { LOG_ERR("Connect failed: %d", err); return; }
    current_conn = bt_conn_ref(conn);
    LOG_INF("BLE connected");
    char msg[16];
    snprintf(msg, sizeof(msg), "VOL:%d", enc_vol);
    bt_nus_send(conn, (uint8_t *)msg, strlen(msg));
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("BLE disconnected: 0x%02x", reason);
    bt_conn_unref(current_conn);
    current_conn = NULL;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = on_connected,
    .disconnected = on_disconnected,
};

static struct bt_nus_cb nus_cb = { .received = on_nus_received };

/* ── main ────────────────────────────────── */
int main(void) {
    int ret;

    /* LED 초기화 */
    for (int i = 0; i < LED_COUNT; i++) {
        if (!gpio_is_ready_dt(&leds[i])) { LOG_ERR("LED%d not ready", i); return -1; }
        gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
    }

    /* 버튼 초기화 */
    for (int i = 0; i < 4; i++) {
        if (!gpio_is_ready_dt(&btns[i])) { LOG_ERR("BTN%d not ready", i); return -1; }
        gpio_pin_configure_dt(&btns[i], GPIO_INPUT);
    }

    /* ADC 초기화 (X축 AIN1 + Y축 AIN2) */
    adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
    if (!device_is_ready(adc_dev)) { LOG_ERR("ADC not ready"); return -1; }
    adc_channel_setup(adc_dev, &joy_x_cfg);
    adc_channel_setup(adc_dev, &joy_y_cfg);
    joy_x_seq.buffer      = &joy_x_buf;
    joy_x_seq.buffer_size = sizeof(joy_x_buf);
    joy_x_seq.resolution  = 12;
    joy_x_seq.channels    = BIT(1);
    joy_y_seq.buffer      = &joy_y_buf;
    joy_y_seq.buffer_size = sizeof(joy_y_buf);
    joy_y_seq.resolution  = 12;
    joy_y_seq.channels    = BIT(2);

    /* QDEC 초기화 */
    qdec_dev = DEVICE_DT_GET(DT_NODELABEL(qdec0));
    if (!device_is_ready(qdec_dev)) {
        LOG_WRN("QDEC not ready"); qdec_dev = NULL;
    } else {
        LOG_INF("QDEC ready");
    }

    /* BLE 초기화 */
    ret = bt_enable(NULL);
    if (ret) { LOG_ERR("bt_enable: %d", ret); return ret; }
    ret = bt_nus_init(&nus_cb);
    if (ret) { LOG_ERR("bt_nus_init: %d", ret); return ret; }
    ret = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (ret) { LOG_ERR("Advertising failed: %d", ret); return ret; }
    LOG_INF("BeatboxMaker ready");

    /* 부팅 LED */
    for (int i = 0; i < LED_COUNT; i++) {
        led_set(i, true); k_msleep(150); led_set(i, false);
    }

    int btn_prev[4] = {0, 0, 0, 0};
    int joy_prev_dir = 0; /* 0=neutral 1=L 2=U 3=D 4=R */

    while (1) {
        k_msleep(POLL_MS);
        char msg[16];

        /* 1. QDEC → 볼륨 */
        if (qdec_dev) {
            struct sensor_value val;
            if (sensor_sample_fetch(qdec_dev) == 0 &&
                sensor_channel_get(qdec_dev, SENSOR_CHAN_ROTATION, &val) == 0) {
                if (val.val1 != 0) {
                    int prev = enc_vol;
                    enc_vol = CLAMP(enc_vol + val.val1 / 18, 0, 100);
                    if (enc_vol != prev) {
                        snprintf(msg, sizeof(msg), "VOL:%d", enc_vol);
                        send_ble(msg);
                        LOG_INF("Vol(enc): %d%%", enc_vol);
                    }
                }
            }
        }

        /* 2. 조이스틱 X축: 좌=GAIN:DOWN / 우=GAIN:UP / 왼쪽+버튼=CLR */
        int joy_left = 0;
        if (adc_read(adc_dev, &joy_x_seq) == 0) {
            int16_t jx = joy_x_buf;
            if (jx < JOY_X_LEFT_MAX) {
                joy_left = 1;
                /* 버튼 동시에 안 눌릴 때만 개별 볼륨 감소 */
                int any_btn = 0;
                for (int k = 0; k < 4; k++)
                    if (gpio_pin_get_dt(&btns[k]) == 1) { any_btn = 1; break; }
                if (!any_btn) send_ble("GAIN:DOWN");
            } else if (jx > JOY_X_RIGHT_MIN && jx < JOY_X_RIGHT_MAX) {
                send_ble("GAIN:UP");
            }
        }

        /* 3. 조이스틱 Y축 (상하 → 탐색) */
        if (adc_read(adc_dev, &joy_y_seq) == 0) {
            int16_t jy = joy_y_buf;

            int dir = 0;
            if      (jy > JOY_Y_UP_MIN && jy < JOY_Y_UP_MAX) dir = 1; /* UP */
            else if (jy < JOY_Y_DOWN_MAX)                      dir = 2; /* DOWN */

            if (dir != joy_prev_dir) {
                if (dir == 1) { send_ble("SEL:UP");   LOG_INF("SEL:UP"); }
                if (dir == 2) { send_ble("SEL:DOWN"); LOG_INF("SEL:DOWN"); }
            }
            joy_prev_dir = dir;
        }

        /* 4. nRF 버튼: 왼쪽+버튼=CLR / 그 외=SLOT */
        for (int i = 0; i < 4; i++) {
            int state = gpio_pin_get_dt(&btns[i]);
            if (btn_prev[i] == 0 && state == 1) {
                if (joy_left) {
                    snprintf(msg, sizeof(msg), "CLR:%d", i + 1);
                    LOG_INF("CLR:%d", i + 1);
                } else {
                    snprintf(msg, sizeof(msg), "SLOT:%d", i + 1);
                    LOG_INF("SLOT:%d", i + 1);
                }
                send_ble(msg);
                led_set(i, true);
                k_msleep(100);
                led_set(i, false);
            }
            btn_prev[i] = state;
        }
    }
}
