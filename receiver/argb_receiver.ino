/*
  ARGB LEDレシーバー
  ESP-NOWで親機から受信したLEDデータを使ってWS2812Bを点灯する
  for M5Stack NanoC6

  Copyright (c) 2025 Kaz  (https://akibabara.com/blog/)
  Released under the MIT license.
  see https://opensource.org/licenses/MIT
*/
#include <M5Unified.h>
#include <WiFi.h>

// GPIO設定
#define LED_GPIO 2        // Groveポート 外付WS2812B LEDデータ出力ピン
#define INLED_GPIO 20     // NanoC6内蔵RGB LED データ出力ピン（WS2812B）
#define INLED_PWR_GPIO 19 // NanoC6内蔵RGB LED 電源制御（HIGHで給電）

// 設定
const uint32_t RECEIVE_TIMEOUT = 10000;  // データ未受信時の消灯までのタイムアウト [ms]
const bool SHOW_INTERNAL_LED = true;  // 内蔵LEDも光らせる
const uint8_t LED_BRIGHTNESS = 255;   // 外付LEDの明るさ(0-255)
const uint8_t INLED_BRIGHTNESS = 64;  // 内蔵LEDの明るさ(0-255)
uint8_t brightness_pattern[] = { 7, 15, 31, 63, 127, 255 };  // 外付LED明るさのパターン（長押し時）

// ESP-NOW関係
#include <esp_now.h>
#define ESP_LED_MAX 150   // ESP-NOWで送信するLEDの最大数
struct RGB_Color {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};
struct EspnowData {
  uint16_t eid;
  uint8_t count;
  uint32_t serial;
  RGB_Color data[ESP_LED_MAX];
};
esp_now_peer_info_t peerInfo;
const uint16_t ESPNOW_ID = 0xA596;  // ESP-NOW ID（識別用、TX/RX側で一致）

// FastLED設定
#include <FastLED.h>
CRGB leds[ESP_LED_MAX];
CRGB inleds[1];

// グローバル変数
EspnowData esp_data;
bool data_ready = false;  // 受信完了フラグ
uint32_t last_receive_serial = 0;
bool first_serial = true;
size_t brightness_index = 0;   // brightness_pattern[]のインデックス
uint8_t my_mac[6];  // 自機のMACアドレス
uint8_t brightness = LED_BRIGHTNESS;  // 外付LEDの明るさ

// デバッグに便利なマクロ定義 --------
#define sp(x) Serial.println(x)
#define spn(x) Serial.print(x)
#define spp(k,v) Serial.println(String(k)+"="+String(v))
#define spf(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#define array_length(x) (sizeof(x) / sizeof(x[0]))


// serialが前回より新しいか（オーバーフロー時は0に戻る想定）
static inline bool isSerialNewer(uint32_t serial, uint32_t last) {
  if (serial == last) return false;
  return (serial - last) <= 0x80000000U;
}

// ESP-NOWの受信コールバック関数
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *receiveData, int data_len) {
  if (esp_now_info == NULL) return;
  // 自分あてのみ処理（ブロードキャスト等の他宛先は破棄）
  if (memcmp(esp_now_info->des_addr, my_mac, 6) != 0) return;
  if (data_len != sizeof(EspnowData)) return;
  memcpy(&esp_data, receiveData, data_len);
  // 前回受信したserial以下なら破棄
  if (!first_serial && !isSerialNewer(esp_data.serial, last_receive_serial)) return;
  // ESP-NOW IDが異なるものは破棄
  if (esp_data.eid != ESPNOW_ID) return;
  first_serial = false;
  last_receive_serial = esp_data.serial;
  data_ready = true;
}

// 受信したデータを元にLEDに点灯する
void sendLed() {
  uint8_t n = (esp_data.count <= ESP_LED_MAX) ? esp_data.count : ESP_LED_MAX;

  // RGB_Color → CRGB にコピー
  for (size_t i = 0; i < n; i++) {
    leds[i].r = esp_data.data[i].r;
    leds[i].g = esp_data.data[i].g;
    leds[i].b = esp_data.data[i].b;
  }
  // 残りは消灯する
  for (size_t i = n; i < ESP_LED_MAX; i++) {
    leds[i] = CRGB::Black;
  }

  // チラつき防止（1は0とみなす）
  // spf("%d, LED0: %3d %3d %3d ,LED1 %3d %3d %3d\n", last_receive_serial, 
  //   leds[0].r, leds[0].g, leds[0].b, leds[1].r, leds[1].g, leds[1].b);
  for (size_t i = 0; i < n; i++) {
    if (leds[i].r == 1) leds[i].r = 0;
    if (leds[i].g == 1) leds[i].g = 0;
    if (leds[i].b == 1) leds[i].b = 0;
  }

  // 外部LED表示
  FastLED[0].showLeds(brightness);

  // 内蔵LED(1個): 先頭のLEDと同じ色
  if (SHOW_INTERNAL_LED) {
    if (esp_data.count > 0) {
      inleds[0].r = esp_data.data[0].r;
      inleds[0].g = esp_data.data[0].g;
      inleds[0].b = esp_data.data[0].b;
    } else {
      inleds[0] = CRGB::Black;
    }
    // 内蔵LED表示
    FastLED[1].showLeds(INLED_BRIGHTNESS);
  }
}

// 全LEDを黒で1回送信（消灯）
void sendAllBlack() {
  for (size_t i = 0; i < ESP_LED_MAX; i++) {
    leds[i] = CRGB::Black;
  }
  inleds[0] = CRGB::Black;
  FastLED[0].showLeds(brightness);
  if (SHOW_INTERNAL_LED) {
    FastLED[1].showLeds(INLED_BRIGHTNESS);
  }
}

// セットアップ
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  delay(500);

  // WiFi設定（自機MACを保存・表示）
  WiFi.mode(WIFI_STA);
  WiFi.macAddress(my_mac);
  spf("Wi-Fi MAC Address: { 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X }\n",
      my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);

  // ESP-NOWの設定
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    sp("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // FastLEDの初期化
  pinMode(LED_GPIO, OUTPUT);
  FastLED.addLeds<WS2812B, LED_GPIO, GRB>(leds, ESP_LED_MAX);  // コントローラ[0]
  FastLED[0].showLeds(brightness);
  if (SHOW_INTERNAL_LED) {
    pinMode(INLED_PWR_GPIO, OUTPUT);
    digitalWrite(INLED_PWR_GPIO, HIGH);  // 内蔵LED電源
    pinMode(INLED_GPIO, OUTPUT);
    FastLED.addLeds<WS2812B, INLED_GPIO, GRB>(inleds, 1);     // コントローラ[1]
    FastLED[1].showLeds(INLED_BRIGHTNESS);
  }
  FastLED.clear();
}

// メインループ
void loop() {
  static uint32_t last_receive = millis();   // 最終受信時刻
  static bool stopped = false;    // タイムアウトで送信停止中
  M5.update();

  // ボタン操作
  if (M5.BtnA.wasReleasefor(500)) {
    // ボタン長押しで輝度変更
    size_t len = sizeof(brightness_pattern) / sizeof(brightness_pattern[0]);
    brightness_index++;
    if (brightness_index >= len) brightness_index = 0;
    brightness = brightness_pattern[brightness_index];
    FastLED[0].showLeds(brightness);
    spf("明るさ変更 %d\n", brightness);
  } else {
    // ボタン短押しで一時停止
    if (M5.BtnA.wasReleased()) {
      sp("一時停止");
      sendAllBlack(); // 全LED黒
      M5.update();
      while (!M5.BtnA.wasReleased()) {
        M5.update();
        delay(10);
      }
      first_serial = true;
    }
  }

  // 受信データがあれば表示
  if (data_ready) {
    last_receive = millis();
    stopped = false;
    sendLed();
    data_ready = false;

  } else if (!stopped && (millis() - last_receive >= RECEIVE_TIMEOUT)) {
    // タイムアウト: 全LED黒を1回送信してから送信停止
    sendAllBlack();
    stopped = true;
    first_serial = true;
  }
  delay(10);
}
