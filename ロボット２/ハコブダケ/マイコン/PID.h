#ifndef PID_H_
#define PID_H_

namespace hakorobo2024
{
  /// @brief PID制御を計算するクラス
  /// @note 各ゲインと制御周期によって計算
  class PIDController
  {
    public:

    /// @brief コンストラクタ
    /// @param kp 比例ゲイン
    /// @param ki 積分ゲイン
    /// @param kd 微分ゲイン
    PIDController(float kp, float ki, float kd);

    /// @brief 計算関数
    /// @param target 目的の値を入れる
    /// @param actual 現在の値、センサからの値などを入れる
    /// @param delta_time 制御周期を入れる。秒[sec]
    float calculation(float target, float actual, float delta_time);

    private:
    float p_gain;
    float i_gain;
    float d_gain;

    float prev_error;
    float integral;
  };
}

#endif