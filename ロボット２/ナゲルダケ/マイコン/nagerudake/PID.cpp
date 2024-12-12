#include "PID.h"

namespace hakorobo2024
{
  PIDController::PIDController(float kp, float ki, float kd) : p_gain(kp), i_gain(ki), d_gain(kd)
  {
    integral = 0.0;
    prev_error = 0.0;
  }

  float PIDController::calculation(float target, float actual, float delta_time)
  {
    float error = target - actual;
    integral += error * delta_time;

    float derivative = (error - prev_error) / delta_time;
    prev_error = error;

    return p_gain * error + i_gain * integral + d_gain * derivative;
  }
}