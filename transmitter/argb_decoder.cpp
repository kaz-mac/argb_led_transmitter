/*
  ARGB LEDデコードCLASS
  for ESP32-S3

  Copyright (c) 2025 Kaz  (https://akibabara.com/blog/)
  Released under the MIT license.
  see https://opensource.org/licenses/MIT
*/
#include "argb_decoder.h"
#include <driver/rmt_rx.h>
#include <string.h>

// コンストラクタ（メンバをゼロ初期化・プロパティの既定値設定）
argb_decoder::argb_decoder()
  : rmt_resolution_hz(10000000),   // 10MHz = 0.1us (100ns)
    rmt_rx_buf_size(1024),
    high_duration_tick(8),         // WS2811B: 0=長いHigh(9tick), 1=短いHigh(6tick)
    rx_gpio_(1),
    max_leds_(0),
    rx_channel_(NULL),
    rx_symbols_(NULL),
    rx_symbol_num_(0),
    receiving_(false),
    decoded_leds_(NULL),
    decoded_led_count_(0) {
}

// デストラクタ（受信バッファ・デコードバッファ解放・RMTチャネル破棄）
argb_decoder::~argb_decoder() {
  if (decoded_leds_) {
    free(decoded_leds_);
    decoded_leds_ = NULL;
  }
  if (rx_symbols_) {
    free(rx_symbols_);
    rx_symbols_ = NULL;
  }
  if (rx_channel_) {
    rmt_disable(rx_channel_);
    rmt_del_channel(rx_channel_);
    rx_channel_ = NULL;
  }
}

// GPIOピン番号を指定して初期化（最大LED数は rmt_rx_buf_size/24 で自動計算）
bool argb_decoder::begin(int rx_gpio) {
  rx_gpio_ = rx_gpio;
  // RMTバッファに収まる最大LED数（1LED = 24シンボル）: rmt_rx_buf_size/24 - 1、ただし1以上
  size_t n = rmt_rx_buf_size / 24;
  max_leds_ = (n > 1) ? (n - 1) : 1;

  // デコード結果用バッファを max_leds_ 分確保（再 begin 時は再確保）
  if (decoded_leds_) {
    free(decoded_leds_);
    decoded_leds_ = NULL;
  }
  decoded_leds_ = (GRB_Color*)malloc(max_leds_ * sizeof(GRB_Color));
  if (decoded_leds_ == NULL) return false;

  if (rx_channel_) {
    return true;  // 既に初期化済み
  }

  rmt_rx_channel_config_t rx_channel_cfg = {};
  rx_channel_cfg.gpio_num = (gpio_num_t)rx_gpio_;
  rx_channel_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
  rx_channel_cfg.resolution_hz = rmt_resolution_hz;
  rx_channel_cfg.mem_block_symbols = 64;
  rx_channel_cfg.flags.invert_in = false;
  rx_channel_cfg.flags.with_dma = false;

  esp_err_t ret = rmt_new_rx_channel(&rx_channel_cfg, &rx_channel_);
  if (ret != ESP_OK) {
    return false;
  }

  rmt_rx_event_callbacks_t cbs = {};
  cbs.on_recv_done = &argb_decoder::rmtRxDoneCallback;

  ret = rmt_rx_register_event_callbacks(rx_channel_, &cbs, this);
  if (ret != ESP_OK) {
    rmt_del_channel(rx_channel_);
    rx_channel_ = NULL;
    return false;
  }

  ret = rmt_enable(rx_channel_);
  if (ret != ESP_OK) {
    rmt_del_channel(rx_channel_);
    rx_channel_ = NULL;
    return false;
  }
  return true;
}

// RMT受信を開始する（受信完了まで次の呼び出しは無視）
bool argb_decoder::startReceive() {
  if (rx_channel_ == NULL) return false;
  if (receiving_) return true;  // 既に受信中

  receiving_ = true;
  if (rx_symbols_ == NULL) {
    rx_symbols_ = (rmt_symbol_word_t*)malloc(rmt_rx_buf_size * sizeof(rmt_symbol_word_t));
    if (rx_symbols_ == NULL) {
      receiving_ = false;
      return false;
    }
  }
  rx_symbol_num_ = 0;

  rmt_receive_config_t receive_config = {};
  receive_config.signal_range_min_ns = 100;
  receive_config.signal_range_max_ns = 3000000;

  esp_err_t ret = rmt_receive(rx_channel_, rx_symbols_,
                              rmt_rx_buf_size * sizeof(rmt_symbol_word_t),
                              &receive_config);
  if (ret != ESP_OK) {
    receiving_ = false;
    return false;
  }
  return true;
}

// 受信がタイムアウトして固まったときに呼ぶ（RMTチャネルを disable/enable でリセット）
void argb_decoder::resetStuckReceive() {
  if (rx_channel_ == NULL) return;
  rmt_disable(rx_channel_);
  rmt_enable(rx_channel_);
  receiving_ = false;
  rx_symbol_num_ = 0;
}

// RMT受信完了コールバック（シンボルを内部バッファにコピー）
void argb_decoder::onRmtRxDone(const rmt_rx_done_event_data_t* edata) {
  rx_symbol_num_ = edata->num_symbols;
  if (rx_symbols_ != NULL && rx_symbol_num_ > 0 && rx_symbol_num_ <= rmt_rx_buf_size) {
    memcpy(rx_symbols_, edata->received_symbols,
           rx_symbol_num_ * sizeof(rmt_symbol_word_t));
  }
  receiving_ = false;
}

// RMTドライバから呼ばれる静的コールバック（user_data の this で onRmtRxDone に転送）
bool argb_decoder::rmtRxDoneCallback(rmt_channel_handle_t channel,
                                    const rmt_rx_done_event_data_t* edata,
                                    void* user_data) {
  argb_decoder* self = (argb_decoder*)user_data;
  if (self) self->onRmtRxDone(edata);
  return false;
}

// シンボル値から High 幅と Low 幅（tick 数）を抽出する
void argb_decoder::extractPulseDurations(uint32_t symbol_val,
                                         uint16_t* high_duration,
                                         uint16_t* low_duration) const {
  uint16_t low_word = symbol_val & 0xFFFF;
  uint16_t high_word = (symbol_val >> 16) & 0xFFFF;
  uint8_t level0 = (low_word >> 15) & 0x01;
  if (level0 == 1) {
    *low_duration = low_word & 0x7FFF;
    *high_duration = high_word & 0x7FFF;
  } else {
    *high_duration = low_word & 0x7FFF;
    *low_duration = high_word & 0x7FFF;
  }
}

// WS2811B の 1 シンボルから 0/1 ビットを判定する（High 幅が閾値未満なら 1）
bool argb_decoder::decodeBit(uint32_t symbol_val) const {
  uint16_t low_word = symbol_val & 0xFFFF;
  uint16_t high_word = (symbol_val >> 16) & 0xFFFF;
  uint8_t level0 = (low_word >> 15) & 0x01;
  uint16_t high_duration = (level0 == 1) ? (high_word & 0x7FFF) : (low_word & 0x7FFF);
  return (high_duration < high_duration_tick);
}

// 受信シンボル列を GRB 24 ビット/LED でデコードし、decoded_leds_ に格納する
void argb_decoder::decodeColorsFromSymbols() {
  rmt_symbol_word_t* symbols = rx_symbols_;
  size_t symbol_num = rx_symbol_num_;
  decoded_led_count_ = 0;
  if (symbols == NULL || decoded_leds_ == NULL || symbol_num < 24) return;

  size_t symbol_idx = 0;
  if (symbol_num > 1) {
    uint16_t high_0 = 0, low_0 = 0, high_1 = 0, low_1 = 0;
    uint32_t val0 = *((uint32_t*)&symbols[0]);
    uint32_t val1 = *((uint32_t*)&symbols[1]);
    extractPulseDurations(val0, &high_0, &low_0);
    extractPulseDurations(val1, &high_1, &low_1);
    if (high_0 < high_duration_tick && high_1 >= high_duration_tick) {
      symbol_idx = 1;
    }
  }

  size_t available_symbols = symbol_num - symbol_idx;
  size_t max_leds = available_symbols / 24;
  if (max_leds > max_leds_) max_leds = max_leds_;

  for (size_t led_idx = 0; led_idx < max_leds; led_idx++) {
    uint8_t g = 0, r = 0, b = 0;
    for (int bit_idx = 0; bit_idx < 24; bit_idx++) {
      if (symbol_idx >= symbol_num) goto done;
      uint32_t symbol_val = *((uint32_t*)&symbols[symbol_idx]);
      bool bit = decodeBit(symbol_val);
      if (bit_idx < 8) {
        g |= (bit ? 1 : 0) << (7 - bit_idx);
      } else if (bit_idx < 16) {
        r |= (bit ? 1 : 0) << (15 - bit_idx);
      } else {
        b |= (bit ? 1 : 0) << (23 - bit_idx);
      }
      symbol_idx++;
    }
    decoded_leds_[decoded_led_count_].g = g;
    decoded_leds_[decoded_led_count_].r = r;
    decoded_leds_[decoded_led_count_].b = b;
    decoded_led_count_++;
  }
done:
  rx_symbol_num_ = 0;  // デコード済みとしてクリア
}

// 未デコードの受信データがあればデコードし、結果を getDecodedLeds() で取得可能にする
void argb_decoder::decode() {
  if (!hasNewData()) return;
  decodeColorsFromSymbols();
}
