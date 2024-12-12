#ifndef I2C_H_
#define I2C_H_

#include <Wire.h>
#include <Arduino.h>

extern char buf[256];
extern int index;

namespace hakorobo2024
{
  /// @brief I2C通信におけるマスター。送信側のクラス
  /// @note A4とA5ピンを用いる。
  class I2C_Transporter
  {
    public:
    /// @brief 通信の初期化を行う
    /// @note setup関数内で実行
    void initialize();

    /// @brief I2Cによる送信を行う
    /// @param msg 送信する文字列
    void transport(uint8_t id, String msg);

    private:
    long PreMillis;
  };

  /// @brief I2C通信におけるスレーブ。受信側のクラス
  /// @note A4とA5ピンを用いる。
  class I2C_Receiver
  {
    public:
    /// @brief 通信の初期化を行う
    /// @note setup関数内で実行
    void initialize(uint8_t id);

    /// @brief I2Cによる受信したデータを取り出す
    /// @note Stringで受信した文字列が返ってくる
    String get_data();
  };
}

/// @brief I2Cによる受信を行う
/// @note initialize時にcallbackとしてセットする
void i2c_receive(int howmany);

#endif