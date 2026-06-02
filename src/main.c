/*
 * Beatbox Maker — nRF52840 DK + Open Smart Rich Shield TWO
 *
 * 볼륨 조절: 인코더 SW(D4=P1.05) 누른 채로 + 조이스틱 좌우(A0=P0.03 ADC)
 *   RIGHT → VOL UP,  LEFT → VOL DOWN
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

/* ── 볼륨 제어 ───────────────────────────── */

/* 조이스틱 ADC: A0 = P0.03 = AIN1 */
#define JOY_LEFT_MAX 300   /* ADC < 300  → LEFT  → VOL DOWN */
#define JOY_RIGHT_MIN 2700 /* ADC 2700~3100 → RIGHT → VOL UP */
#define JOY_RIGHT_MAX 3100 /* 중립값(~3318)과 구분 */
#define VOL_STEP 2         /* 150ms마다 변화량 */
#define VOL_INTERVAL 150   /* ms */

static int enc_vol = 50;
static const struct device *gpio1_dev;
static const struct device *qdec_dev;

/* ── ADC: 조이스틱(AIN1) + MIC(AIN2) ─────── */
static const struct device *adc_dev;

static int16_t joy_buf;
static struct adc_sequence joy_seq;
static const struct adc_channel_cfg joy_ch_cfg = {
    .gain = ADC_GAIN_1_4,
    .reference = ADC_REF_VDD_1_4,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = 1,
    .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput1,
};

static int16_t mic_buf;
static struct adc_sequence mic_seq = {
    .buffer = &mic_buf,
    .buffer_size = sizeof(mic_buf),
    .resolution = 12,
    .channels = BIT(2),
};
static const struct adc_channel_cfg mic_ch_cfg = {
    .gain = ADC_GAIN_1_4,
    .reference = ADC_REF_VDD_1_4,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = 2,
    .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput2,
};

K_MUTEX_DEFINE(adc_mutex);

/* ── BLE ─────────────────────────────────── */
static struct bt_conn *current_conn;
static bool recording;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, "BeatboxMaker", sizeof("BeatboxMaker") - 1),
};

/* ── 유틸 ────────────────────────────────── */
static void send_ble(const char *msg) {
  if (current_conn)
    bt_nus_send(current_conn, (uint8_t *)msg, strlen(msg));
}

static void led_set(int idx, bool on) {
  if (idx < 0 || idx >= LED_COUNT)
    return;
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

  if (strncmp(cmd, "REC:START", 9) == 0) {
    recording = true;
  } else if (strncmp(cmd, "REC:STOP", 8) == 0) {
    recording = false;
  } else if (strncmp(cmd, "PLAY:", 5) == 0) {
    int slot = atoi(&cmd[5]);
    if (slot >= 1 && slot <= 4) {
      for (int i = 0; i < LED_COUNT; i++)
        led_set(i, i == slot - 1);
      k_msleep(1500);
      led_set(slot - 1, false);
    }
  }
}

static void on_connected(struct bt_conn *conn, uint8_t err) {
  if (err) {
    LOG_ERR("Connect failed: %d", err);
    return;
  }
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
  recording = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

static struct bt_nus_cb nus_cb = {.received = on_nus_received};

/* ── MIC 스레드 ──────────────────────────── */
K_THREAD_STACK_DEFINE(mic_stack, 1024);
static struct k_thread mic_thread;

static void mic_thread_fn(void *a, void *b, void *c) {
  char msg[24];
  while (1) {
    if (recording && current_conn) {
      if (k_mutex_lock(&adc_mutex, K_MSEC(1)) == 0) {
        if (adc_read(adc_dev, &mic_seq) == 0) {
          snprintf(msg, sizeof(msg), "MIC:%d", (int)mic_buf);
          bt_nus_send(current_conn, (uint8_t *)msg, strlen(msg));
        }
        k_mutex_unlock(&adc_mutex);
      }
      k_busy_wait(125);
    } else {
      k_msleep(10);
    }
  }
}

/* ── main ────────────────────────────────── */
int main(void) {
  int ret;

  /* LED 초기화 */
  for (int i = 0; i < LED_COUNT; i++) {
    if (!gpio_is_ready_dt(&leds[i])) {
      LOG_ERR("LED%d not ready", i);
      return -1;
    }
    gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
  }

  /* ADC 초기화 (조이스틱 AIN1 + MIC AIN2) */
  adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
  if (!device_is_ready(adc_dev)) {
    LOG_ERR("ADC not ready");
    return -1;
  }
  adc_channel_setup(adc_dev, &joy_ch_cfg);
  adc_channel_setup(adc_dev, &mic_ch_cfg);
  joy_seq.buffer = &joy_buf;
  joy_seq.buffer_size = sizeof(joy_buf);
  joy_seq.resolution = 12;
  joy_seq.channels = BIT(1);

  /* BLE 초기화 */
  ret = bt_enable(NULL);
  if (ret) {
    LOG_ERR("bt_enable: %d", ret);
    return ret;
  }
  ret = bt_nus_init(&nus_cb);
  if (ret) {
    LOG_ERR("bt_nus_init: %d", ret);
    return ret;
  }
  ret = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (ret) {
    LOG_ERR("Advertising failed: %d", ret);
    return ret;
  }
  LOG_INF("BeatboxMaker ready");

  /* QDEC 초기화 */
  qdec_dev = DEVICE_DT_GET(DT_NODELABEL(qdec0));
  if (!device_is_ready(qdec_dev)) {
    LOG_WRN("QDEC not ready — 인코더 비활성화");
    qdec_dev = NULL;
  } else {
    LOG_INF("QDEC ready");
  }

  /* MIC 스레드 */
  k_thread_create(&mic_thread, mic_stack, 1024, mic_thread_fn, NULL, NULL, NULL,
                  7, 0, K_NO_WAIT);
  k_thread_name_set(&mic_thread, "mic");

  /* 부팅 LED */
  for (int i = 0; i < LED_COUNT; i++) {
    led_set(i, true);
    k_msleep(150);
    led_set(i, false);
  }

  /* 볼륨 제어 루프: 조이스틱(좌우) + QDEC 인코더 */
  while (1) {
    k_msleep(VOL_INTERVAL);
    int changed = 0;

    /* QDEC 인코더 읽기 (하드웨어 디코더) */
    if (qdec_dev != NULL) {
      struct sensor_value val;
      if (sensor_sample_fetch(qdec_dev) == 0 &&
          sensor_channel_get(qdec_dev, SENSOR_CHAN_ROTATION, &val) == 0) {
        if (val.val1 != 0) {
          enc_vol = CLAMP(enc_vol + val.val1 / 18, 0, 100);
          changed = 1;
        }
      }
    }

    /* 조이스틱 ADC 읽기 */
    int16_t joy = 0;
    if (k_mutex_lock(&adc_mutex, K_MSEC(5)) != 0)
      continue;
    int adc_ret = adc_read(adc_dev, &joy_seq);
    if (adc_ret == 0)
      joy = joy_buf;
    k_mutex_unlock(&adc_mutex);
    if (adc_ret == 0) {
      if (joy < JOY_LEFT_MAX) {
        enc_vol = MAX(0, enc_vol - VOL_STEP);
        changed = 1;
      } else if (joy > JOY_RIGHT_MIN && joy < JOY_RIGHT_MAX) {
        enc_vol = MIN(100, enc_vol + VOL_STEP);
        changed = 1;
      }
    }

    if (changed) {
      char msg[16];
      snprintf(msg, sizeof(msg), "VOL:%d", enc_vol);
      send_ble(msg);
      LOG_INF("Vol: %d%%", enc_vol);
    }
  }
}
