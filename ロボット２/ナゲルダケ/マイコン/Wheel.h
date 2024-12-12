#ifndef WHEEL_H_
#define WHEEL_H_

namespace hakorobo2024
{
  class Mecanum4
  {
    public:
    void set_vec(float x, float y, float rotation);

    float get_fl();
    float get_fr();
    float get_rl();
    float get_rr();

    private:
    float x_, y_, rotation_;
    const float rate = 0.707106781;
  };

  class Wheel2
  {
    public:
    void set_vec(float y, float rotation);

    float get_l();
    float get_r();

    private:
    float y_, rotation_;
    const float y_rate = 0.5;
    const float rotation_rate = 0.5;
  };
}

#endif