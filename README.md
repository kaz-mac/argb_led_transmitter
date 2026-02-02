# ARGB LED Transmitter / Receiver

マザーボード上のARGB LED (WS2812B)のデータ信号を解析して、ESP-NOWで子機へ送信するトランスミッターと、受信したデータをARGB LEDに表示するデバイスです。

## ハードウェア

### 親機（トランスミッター）

M5Stack AtomS3想定。ESP32-S3ならたぶん動く。

### 子機（レシーバー）

M5Stack NanoC6想定。たぶんどれでも動く。

## 設定

宛先の子機のMACアドレスを指定する。子機のMACアドレスは起動時にシリアルに出力されるのでそれをコピペする。

```cpp argb_transmitter.ino
// ESP-NOW送信先MACアドレス
static const uint8_t targetAddresses[][6] = {
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },   // 子機1
  // { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF },   // 子機2（必要に応じて追加）
};
```
その他の設定はソースコード参照。

## 接続

マザーボードの出力はTTLレベルなので、親機は抵抗で分圧するかレベル変換用のバッファーを通す。直接繋いじゃだめ。子機は直接LEDに繋いでもたぶん動く。
