#include "Wheel.h"

namespace hakorobo2024
{
  void Mecanum4::set_vec(float x, float y, float rotation)
  {
    x_ = x;
    y_ = y;
    rotation_ = rotation;
  }

  float Mecanum4::get_fl()
  {
    return x_*rate + y_*rate + 0.5 * rotation_;
  }

  float Mecanum4::get_fr()
  {
    return x_*rate - y_*rate + 0.5 * rotation_;
  }

  float Mecanum4::get_rl()
  {
    return -x_*rate + y_*rate + 0.5 * rotation_;
  }

  float Mecanum4::get_rr()
  {
    return -x_*rate - y_*rate + 0.5 * rotation_;
  }

  void Wheel2::set_vec(float y, float rotation)
  {
    y_ = y;
    rotation_ = rotation;
  }

  float Wheel2::get_l()
  {
    return -y_*y_rate - rotation_*rotation_rate;
  }

  float Wheel2::get_r()
  {
    return y_*y_rate - rotation_*rotation_rate;
  }
}