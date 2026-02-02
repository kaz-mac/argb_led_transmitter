/*
  ARGB LEDトランスミッター
  WS2812Bのデータ信号を解析してESP-NOWで子機へ送信する
  for M5Stack AtomS3 (ESP32-S3専用)

  Copyright (c) 2025 Kaz  (https://akibabara.com/blog/)
  Released under the MIT license.
  see https://opensource.org/licenses/MIT
*/
#include <Arduino.h>
#include <M5Unified.h>
#include <esp_log.h>
#include <math.h>
#include "argb_decoder.h"
#include <WiFi.h>

// GPIO設定
#define RMT_RX_GPIO 1   // Groveポート WS2812B LEDデータ信号入力ピン
#define INLED_GPIO 35   // 内蔵RGB LED データ出力ピン (AtomS3 Liteの場合)
const uint8_t INLED_BRIGHTNESS = 64;  // 内蔵LEDの明るさ(0-255)

// 設定
#define NUM_LEDS 215    // LEDの最大数（decoder.rmt_rx_buf_size / 24 より大きくする）
const bool PREVIEW_LCD = false; // LCDに受信したカラーをプレビューする（LCD付き専用）
const bool PREVIEW_LED = true;  // 内蔵LEDに受信したカラーをプレビューする（Lite専用）

// ESP-NOW送信先MACアドレス
static const uint8_t targetAddresses[][6] = {
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },   // 子機1
  // { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF },   // 子機2（必要に応じて追加）
};
const uint16_t ESPNOW_ID = 0xA596;  // ESP-NOW ID（識別用、TX/RX側で一致）

// FastLED設定
#include <FastLED.h>
CRGB inleds[1];

// インスタンス
argb_decoder decoder;
M5Canvas canvas(&M5.Display);

// グローバル変数（メイン用）
int m5_w, m5_h;
size_t calibrated_count = 0;


// ESP-NOW関係
#include <esp_now.h>
#define ESP_LED_MAX 150   // ESP-NOWで送信するLEDの最大数
struct RGB_Color {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};
struct EspnowData{
  uint16_t eid;
  uint8_t count;
  uint32_t serial;
  RGB_Color data[ESP_LED_MAX];
};
esp_now_peer_info_t peerInfo;
#define ESP_NOW_PEER_COUNT (sizeof(targetAddresses) / sizeof(targetAddresses[0]))


// デバッグに便利なマクロ定義 ESP32-S3M5.Log対応ver --------
#define sp(x) M5.Log.printf("%s\n", String(x).c_str())
#define spn(x) M5.Log.printf("%s\n", String(x).c_str())  //【非推薦】M5.Logでは改行が必須
#define spp(k,v) M5.Log.printf("%s=%s\n", k, String(v).c_str())
#define spf(fmt, ...) M5.Log.printf(fmt, __VA_ARGS__)
#define array_length(x) (sizeof(x) / sizeof(x[0]))


// ----------------------------------------------------------------------------------
// ESP-NOW送信コールバック関数
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    // sp("ESP-NOW送信成功");
  } else {
    // sp("ESP-NOW送信失敗");
  }
}

// デコードしたLED色データをESP-NOWで送信する
void sendEspnow(const GRB_Color* leds, size_t count) {
  if (leds == NULL || count == 0) return;
  if (ESP_NOW_PEER_COUNT == 0) return;
  
  // 送信するLED数をESP_LED_MAXの範囲内に制限
  size_t send_count = (count > ESP_LED_MAX) ? ESP_LED_MAX : count;
  EspnowData espnow_data;
  static uint32_t serial = 0;
  espnow_data.eid = ESPNOW_ID;
  espnow_data.serial = serial++;
  espnow_data.count = (uint8_t)send_count;
  
  // GRB_ColorからRGB_Colorへ変換してespnow_data.data配列にコピー
  for (size_t i = 0; i < send_count; i++) {
    espnow_data.data[i].r = leds[i].r;
    espnow_data.data[i].g = leds[i].g;
    espnow_data.data[i].b = leds[i].b;
  }
  
  // 登録済みの全送信先に送信
  for (size_t p = 0; p < ESP_NOW_PEER_COUNT; p++) {
    esp_err_t result = esp_now_send(targetAddresses[p], (uint8_t*)&espnow_data, sizeof(espnow_data));
    if (result != ESP_OK) {
      // spf("ESP-NOW送信エラー [%d]: %d\n", (int)p, result);
    }
  }
}

// 内蔵LEDに先頭1個のLED色をプレビュー表示する
void previewLed(const GRB_Color* leds) {
  if (leds == NULL) return;
  inleds[0].r = leds[0].r;
  inleds[0].g = leds[0].g;
  inleds[0].b = leds[0].b;
  FastLED.show();
}

// ----------------------------------------------------------------------------------

// 2つのLED配列を比較する（差は+/-1まで許容）
static bool ledsDiffer(const GRB_Color* a, const GRB_Color* b, size_t count) {
  for (size_t i = 0; i < count; i++) {
    if (abs((int)a[i].g - (int)b[i].g) > 1) return true;
    if (abs((int)a[i].r - (int)b[i].r) > 1) return true;
    if (abs((int)a[i].b - (int)b[i].b) > 1) return true;
  }
  return false;
}

// デコードしたLED色をLCDにグリッド表示する（個数に応じて縦横分割数を自動決定）
void drawDecodedLedsSquare(const GRB_Color* leds, size_t count) {
  if (leds == NULL || count == 0 || m5_w <= 0 || m5_h <= 0) return;

  // 収まる最小の正方形グリッド数: n個 → ceil(sqrt(n)) x ceil(sqrt(n))
  size_t grid = 1;
  while (grid * grid < count) grid++;

  int cw = m5_w / (int)grid;   // 1マス幅
  int ch = m5_h / (int)grid;   // 1マス高さ
  if (cw <= 0 || ch <= 0) return;

  canvas.fillScreen(TFT_BLACK);
  for (size_t i = 0; i < count; i++) {
    int col = (int)(i % grid);
    int row = (int)(i / grid);
    int x = col * cw;
    int y = row * ch;
    uint16_t color = canvas.color565(leds[i].r, leds[i].g, leds[i].b);
    canvas.fillRect(x, y, cw, ch, color);
  }
  canvas.pushSprite(0, 0);
}

// デコードしたLED色の先頭12個を時計周りでLCDに表示する
#define CIRCULAR_LED_COUNT 12
void drawDecodedLedsCircle(const GRB_Color* leds, size_t count) {
  if (leds == NULL || m5_w <= 0 || m5_h <= 0) return;

  const int cx = m5_w / 2;
  const int cy = m5_h / 2;
  const int min_half = (m5_w < m5_h) ? m5_w / 2 : m5_h / 2;
  const int margin = 4;
  const int led_radius = (min_half - margin) / 6;
  const int circle_r = min_half - led_radius - margin;  // LED中心の円の半径

  canvas.fillScreen(TFT_BLACK);

  for (int i = 0; i < CIRCULAR_LED_COUNT; i++) {
    const float angle_deg = -90.0f + (float)i * (360.0f / (float)CIRCULAR_LED_COUNT);
    const float angle_rad = angle_deg * 3.14159265f / 180.0f;
    const int x = cx + (int)(circle_r * cosf(angle_rad));
    const int y = cy + (int)(circle_r * sinf(angle_rad));

    uint16_t color;
    if ((size_t)i < count) {
      color = canvas.color565(leds[i].r, leds[i].g, leds[i].b);
    } else {
      color = TFT_BLACK;
    }
    canvas.fillCircle(x, y, led_radius, color);
  }
  canvas.pushSprite(0, 0);
}

// 受信データのキャリブレーション
typedef struct {
  size_t led_count;
  uint16_t occurrences;
} calibration_entry_t;
#define CALIBRATION_MAX_ENTRIES 32  // 登録するLED個数の種類の上限（通常は数種類程度）

// 指定回数だけデータを受信し、LED個数の出現回数を集計する
size_t signalCalibration(size_t receive_count) {
  static calibration_entry_t entries[CALIBRATION_MAX_ENTRIES];
  size_t num_entries = 0;

  // 集計用エントリの初期化
  for (size_t e = 0; e < CALIBRATION_MAX_ENTRIES; e++) {
    entries[e].led_count = 0;
    entries[e].occurrences = 0;
  }

  // 指定回数だけ受信を繰り返す
  for (size_t i = 0; i < receive_count; i++) {
    decoder.startReceive();
    const uint32_t timeout_ms = 500;
    uint32_t deadline = millis() + timeout_ms;
    while (!decoder.hasNewData() && (millis() < deadline)) {
      delay(10);
    }
    if (!decoder.hasNewData()) continue;  // タイムアウト時はスキップ

    decoder.decode();
    size_t n = decoder.getDecodedLedCount();

    // 既存エントリで同じLED個数を検索
    size_t found = CALIBRATION_MAX_ENTRIES;
    for (size_t e = 0; e < num_entries; e++) {
      if (entries[e].led_count == n) {
        found = e;
        break;
      }
    }
    // 既存なら出現回数を加算、未登録かつ空きがあれば新規登録
    if (found < CALIBRATION_MAX_ENTRIES) {
      entries[found].occurrences++;
    } else if (num_entries < CALIBRATION_MAX_ENTRIES) {
      entries[num_entries].led_count = n;
      entries[num_entries].occurrences = 1;
      num_entries++;
    }
  }

  // 出現回数が最大のLED個数を選んで返す（結果の集計）
  size_t best_count = 0;
  uint16_t best_val = 0;
  for (size_t e = 0; e < num_entries; e++) {
    spf("%d回: LED %d個\n", entries[e].occurrences, entries[e].led_count);
    if (entries[e].occurrences > best_val) {
      best_val = entries[e].occurrences;
      best_count = entries[e].led_count;
    }
  }
  return best_count;
}

// ----------------------------------------------------------------------------------

// セットアップ
void setup() {
  // 初期化
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  m5_w = M5.Display.width();
  m5_h = M5.Display.height();

  // ログ設定
  M5.Log.setLogLevel(m5::log_target_serial, ESP_LOG_DEBUG);
  M5.Log.setEnableColor(m5::log_target_serial, false);
  delay(1000);

  // ディスプレイの設定
  M5.Display.setTextSize(2);
  M5.Display.setTextScroll(false);

  // キャンバスの作成
  canvas.setColorDepth(16);
  canvas.createSprite(m5_w, m5_h);

  // RMTの「user buffer too small, received symbols truncated」ログを抑制
  esp_log_level_set("rmt", ESP_LOG_NONE);

  // WiFiとESP-NOWの設定
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    sp("ESP-NOW initialize error");
    return;
  }
  
  // ESP-NOWの設定
  esp_now_register_send_cb(OnDataSent);
  peerInfo.channel = 0;   // 自動チャンネル選択
  peerInfo.encrypt = false;  // 暗号化なし
  for (size_t i = 0; i < ESP_NOW_PEER_COUNT; i++) {
    memcpy(peerInfo.peer_addr, targetAddresses[i], 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      spf("ESP-NOWピア追加エラー [%d]\n", (int)i);
    }
  }

  // ARGBデコーダーの初期化
  decoder.rmt_rx_buf_size = 5120;   // LED 213個相当分のバッファー (5120/24=213)
  pinMode(RMT_RX_GPIO, INPUT);
  if (!decoder.begin(RMT_RX_GPIO)) {
    sp("argb_decoder 初期化エラー");
  }

  // FastLEDの初期化
  if (PREVIEW_LED) {
    pinMode(INLED_GPIO, OUTPUT);
    FastLED.addLeds<WS2812B, INLED_GPIO, GRB>(inleds, 1);
    FastLED.setBrightness(INLED_BRIGHTNESS);
    FastLED.clear();
    FastLED.show();
  }
}

// ----------------------------------------------------------------------------------

// メインループ
void loop() {
  static int error_count = 0; // デコードエラー回数
  static uint32_t last_update_time = millis();  // 最終描画更新時刻
  static GRB_Color prev_leds[NUM_LEDS];
  static size_t prev_led_count = 0;
  M5.update();

  // 自動キャリブレーション
  if (calibrated_count == 0) {
    sp("キャリブレーション開始");
    calibrated_count = signalCalibration(100);
    if (calibrated_count > 0) {
      spf("最多出現LED個数 = %d個\n", calibrated_count);
    } else {
      sp("キャリブレーション失敗");
    }
    error_count = 0;
    last_update_time = millis();
  }

  // ボタン押下で一時停止
  bool pause = M5.BtnA.wasReleased();

  // 受信データがあればデコード（pause のときは黒データで送信）
  if (decoder.hasNewData() || pause) {
    const GRB_Color* leds;
    size_t n;
    if (pause) {
      // 全色を黒にする
      n = (calibrated_count > 0 && calibrated_count <= NUM_LEDS) ? calibrated_count : 1;
      for (size_t i = 0; i < n && i < NUM_LEDS; i++) prev_leds[i] = { 0, 0, 0 };
      leds = prev_leds;
    } else {
      // デコードする
      decoder.decode();
      leds = decoder.getDecodedLeds();
      n = decoder.getDecodedLedCount();
    }

    // デコード成功時、または pause 時
    if (n == calibrated_count || pause) {
      // 前回の受信結果と異なる場合のみ描画を更新
      bool changed = (n != prev_led_count) || (n > NUM_LEDS);
      if (!changed) changed = (millis() - last_update_time > 3000);
      if (!changed) changed = ledsDiffer(leds, prev_leds, n);
      if (changed || pause) {  // pause 時は必ず表示・送信する
        // 表示とデータ送信
        drawDecodedLedsSquare(leds, n);   // LCDに表示（正方形）
        // drawDecodedLedsCircle(leds, n);   // LCDに表示（円形）
        if (PREVIEW_LED) previewLed(leds);  // 内蔵LEDに表示
        sendEspnow(leds, n);  // ESP-NOWで送信
        // 後処理
        for (size_t i = 0; i < n && i < NUM_LEDS; i++) prev_leds[i] = leds[i];
        prev_led_count = n;
      } else {
        sp("色変化なし");
      }
      error_count = 0;
      last_update_time = millis();
      // 一時停止
      if (pause) {
        M5.update();
        while (!M5.BtnA.wasReleased()) {
          M5.update();
          delay(10);
        }
      }
    } else {
      error_count++;
      spf("error=%d (n=%d)\n", error_count, n);
    }
  }

  // エラーが連続100回以上 or 無信号5秒以上になったら再キャリブレーション
  if (error_count > 100 || millis() - last_update_time > 5000) {
    calibrated_count = 0;
    error_count = 0;
  }

  // 受信開始
  decoder.startReceive();
  delay((error_count == 0) ? 55 : 9);
}
