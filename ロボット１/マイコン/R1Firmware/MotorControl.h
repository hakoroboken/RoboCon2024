#ifndef MOTORCONTROL_HPP_
#define MOTORCONTROL_HPP_

namespace r1_firmware
{
  class MotorHandler
  {
    public:
    void setup();

    void control_front(int pwm);
    void control_rear_left(int pwm);
    void control_rear_right(int pwm);
    void control_valve(int pwm, bool switch_1, bool switch_2);

    bool limit_switch_1();
    bool limit_switch_2();

    private:
    int prev_limit_1;
    int prev_limit_2;
  };
}

#endif