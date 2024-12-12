#ifndef UTILS_H_
#define UTILS_H_

#include <Arduino.h>

#define RPM2AMPARE 0.29411764705882354

namespace hakorobo2024
{
  /// @brief 同じ比率で変換する関数
  /// @param T 入力変数の型
  /// @param U 出力変数の型
  /// @note 各引数はその名前のまんま
  template<typename T, typename U>
  U remap(T x, T in_min, T in_max, U out_min, U out_max)
  {
    return (x - in_min) * ((T)(out_max - out_min)) / (in_max - in_min) + out_min;
  }

  /// @brief RPMを出力電流値に変換する関数
  /// @param T RPMの型
  /// @note ロボマスター用であるため16bitのint型
  template<typename T>
  int16_t rpm2ampare(T rpm)
  {
    return (int)(rpm * RPM2AMPARE);
  }
}

#endif