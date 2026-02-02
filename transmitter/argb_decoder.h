/*
  ARGB LEDデコードCLASS
  for ESP32-S3

  Copyright (c) 2025 Kaz  (https://akibabara.com/blog/)
  Released under the MIT license.
  see https://opensource.org/licenses/MIT
*/
#ifndef ARGB_DECODER_H
#define ARGB_DECODER_H

#include <Arduino.h>
#include <driver/rmt_rx.h>

// LED色データ（GRB順）
struct GRB_Color {
  uint8_t g;  // Green
  uint8_t r;  // Red
  uint8_t b;  // Blue
};

class argb_decoder {
public:
  // 変更可能なプロパティ（begin() の前に設定すること）
  uint32_t rmt_resolution_hz;  // RMT分解能 [Hz] 既定: 10000000 (0.1us)
  size_t rmt_rx_buf_size;      // 受信バッファシンボル数 既定: 1024（最大LED数 = 本値/24-1 で自動計算）
  uint16_t high_duration_tick; // 0/1ビット閾値 [tick] 既定: 8 (WS2811B: 0=長いHigh, 1=短いHigh)

  argb_decoder();
  ~argb_decoder();

  // GPIOピン番号を指定して初期化（最大LED数は rmt_rx_buf_size/24 で自動計算）
  bool begin(int rx_gpio = 5);

  // 受信を開始する。受信完了まで次の startReceive は無視される
  bool startReceive();

  // 受信中かどうか
  bool isReceiving() const { return receiving_; }

  // 受信がタイムアウトして固まったときに呼ぶ。RMTチャネルをリセットし次の startReceive() が可能になる
  void resetStuckReceive();

  // 受信完了して未デコードのデータがあるか
  bool hasNewData() const { return !receiving_ && rx_symbol_num_ > 0; }

  // 受信済みシンボルをデコードし、結果を内部配列に保存する。
  // hasNewData() が true のときだけ呼ぶ。
  void decode();

  // デコード結果の取得（decode() 実行後）
  const GRB_Color* getDecodedLeds() const { return decoded_leds_; }
  size_t getDecodedLedCount() const { return decoded_led_count_; }

private:
  int rx_gpio_;
  size_t max_leds_;  // rmt_rx_buf_size/24 から自動計算したデコードする最大LED数
  rmt_channel_handle_t rx_channel_;
  rmt_symbol_word_t* rx_symbols_;
  volatile size_t rx_symbol_num_;   // コールバックとメインループで共有（volatile で可視性を保証）
  volatile bool receiving_;
  GRB_Color* decoded_leds_;  // max_leds_ 分を begin() で確保
  size_t decoded_led_count_;

  void extractPulseDurations(uint32_t symbol_val, uint16_t* high_duration, uint16_t* low_duration) const;
  bool decodeBit(uint32_t symbol_val) const;
  void decodeColorsFromSymbols();

  void onRmtRxDone(const rmt_rx_done_event_data_t* edata);
  static bool rmtRxDoneCallback(rmt_channel_handle_t channel,
                                const rmt_rx_done_event_data_t* edata,
                                void* user_data);
};

#endif // ARGB_DECODER_H
