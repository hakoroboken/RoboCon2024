#include "MotorControl.h"

#include <Arduino.h>

using namespace r1_firmware;

void MotorHandler::setup()
{
  prev_limit_1 = 0;
  prev_limit_2 = 0;

  // Debug
  pinMode(LED_BUILTIN, OUTPUT);

  // Front
  // PWM 2
  pinMode(3, OUTPUT);

  // RearLeft
  // PWM 4
  pinMode(5, OUTPUT);

  // RearRight
  // PWM 6
  pinMode(7, OUTPUT);

  // Valve
  // PWM 8
  pinMode(9, OUTPUT);

  // LimitSwitch Voltage
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);

  pinMode(26, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);

  Serial.println("[MotorHandler] MotorHandler OK!!!!!!");
}

void MotorHandler::control_front(int pwm)
{
  int pwm_ = 0;

  if(pwm > 255)
  {
    pwm_ = 255;
  }
  else
  {
    pwm_ = pwm;
  }

  if(pwm_ > 0)
  {
    digitalWrite(3, LOW);
    analogWrite(2, pwm_);
  }
  else if(pwm_ < 0)
  {
    digitalWrite(3, HIGH);
    analogWrite(2, (-1*pwm_));
  }
  else
  {
    digitalWrite(3, LOW);
    analogWrite(2, 0);
  }
}

void MotorHandler::control_rear_left(int pwm)
{
  int pwm_ = 0;

  if(pwm > 255)
  {
    pwm_ = 255;
  }
  else
  {
    pwm_ = pwm;
  }

  if(pwm_ > 0)
  {
    digitalWrite(5, LOW);
    analogWrite(4, pwm_);
  }
  else if(pwm_ < 0)
  {
    digitalWrite(5, HIGH);
    analogWrite(4, (-1*pwm_));
  }
  else
  {
    digitalWrite(5, LOW);
    analogWrite(4, 0);
  }
}

void MotorHandler::control_rear_right(int pwm)
{
  int pwm_ = 0;

  if(pwm > 255)
  {
    pwm_ = 255;
  }
  else
  {
    pwm_ = pwm;
  }

  if(pwm_ > 0)
  {
    digitalWrite(7, LOW);
    analogWrite(6, pwm_);
  }
  else if(pwm < 0)
  {
    digitalWrite(7, HIGH);
    analogWrite(6, (-1*pwm_));
  }
  else
  {
    digitalWrite(7, LOW);
    analogWrite(6, 0);
  }
}

void MotorHandler::control_valve(int pwm, bool switch_1, bool switch_2)
{
  if(pwm > 0)
  {
    if(!switch_1)
    {
      digitalWrite(9, LOW);
      analogWrite(8, pwm);
    }
    else
    {
      digitalWrite(9, LOW);
      analogWrite(8, 0);
    }
  }
  else if(pwm < 0)
  {
    if(!switch_2)
    {
      digitalWrite(9, HIGH);
      analogWrite(8, -1*pwm);
    }
    else
    {
      digitalWrite(9, LOW);
      analogWrite(8, 0);
    }
  }
  else
  {
    digitalWrite(9, LOW);
    analogWrite(8, 0);
  }
}

bool MotorHandler::limit_switch_1()
{
  uint8_t read_value = analogRead(26);

  if(read_value != 255 && prev_limit_1 != 255)
  {
    prev_limit_1 = read_value;
    return true;
  }
  else
  {
    prev_limit_1 = read_value;
    return false;
  }
}

bool MotorHandler::limit_switch_2()
{
  uint8_t read_value = analogRead(27);

  if(read_value != 255 && prev_limit_2 != 255)
  {
    prev_limit_2 = read_value;
    return true;
  }
  else
  {
    prev_limit_2 = read_value;
    return false;
  }
}
