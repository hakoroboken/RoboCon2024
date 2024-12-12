#ifndef LOOP_CYCLER
#define LOOP_CYCLER

#include "Arduino.h"

namespace hakorobo2024
{
  /// @brief Loop周期を固定するクラス
  /// @note delayを使わずにloop周期を変えることができる
  class LoopCycler
  {
    public:

    /// @brief コンストラクタ
    /// @param delta_ms 周期をミリ秒で指定する
    LoopCycler(long delta_ms);

    /// @brief ループ内で回す関数
    /// @note 指定された時間だけ周期を待つ
    void cycle();

    /// @brief ループ周期を秒[sec]で獲得する関数
    /// @note PIDなど計算時に用いる。
    float getDeltaTime();

    private:
    long delta_ms_;
    long prev_time_;
  };

}

#endif