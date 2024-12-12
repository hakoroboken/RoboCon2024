#include "RoboMaster.h"
#include <Arduino.h>

namespace hakorobo2024
{
  RoboMaster::Cmd::Cmd():
  buf_1({0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}),
  buf_2({0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})
  {

  }

  RoboMaster::FeedBack::FeedBack():
  buf({0}),
  ID(0x00)
  {

  }

  void RoboMaster::setType(uint8_t id, Type type)
  {
    type_[id-1] = type;
  }

  void RoboMaster::setCurrent(uint8_t id, int ampare)
  {
    output[id-1] = ampare;
    output[id-1] = constrainCurrent(id-1);

    // Serial.println(output[id-1]);
  }

  RoboMaster::Cmd RoboMaster::createCommand()
  {
    auto cmd = RoboMaster::Cmd::Cmd();

    cmd.buf_1[0] = (output[0] >> 8) & 0xFF;
    cmd.buf_1[1] = output[0] & 0xFF;
    cmd.buf_1[2] = (output[1] >> 8) & 0xFF;
    cmd.buf_1[3] = output[1] & 0xFF;
    cmd.buf_1[4] = (output[2] >> 8) & 0xFF;
    cmd.buf_1[5] = output[2] & 0xFF;
    cmd.buf_1[6] = (output[3] >> 8) & 0xFF;
    cmd.buf_1[7] = output[3] & 0xFF;

    cmd.buf_2[0] = (output[4] >> 8) & 0xFF;
    cmd.buf_2[1] = output[4] & 0xFF;
    cmd.buf_2[2] = (output[5] >> 8) & 0xFF;
    cmd.buf_2[3] = output[5] & 0xFF;
    cmd.buf_2[4] = (output[6] >> 8) & 0xFF;
    cmd.buf_2[5] = output[6] & 0xFF;
    cmd.buf_2[6] = (output[7] >> 8) & 0xFF;
    cmd.buf_2[7] = output[7] & 0xFF;

    return cmd;
  }

  void RoboMaster::setFeedBack(FeedBack data)
  {
    int16_t angle_data = data.buf[0] << 8 | data.buf[1];
    int16_t rpm_data = data.buf[2] << 8 | data.buf[3];
    int16_t amp = data.buf[4] << 8 | data.buf[5];
    int8_t  temp = data.buf[6];

    int16_t angle = hakorobo2024::remap(angle_data, 0, 8192, 0, 360);

    if(data.ID == 0x201)
    {
      rpm_[0] = rpm_data;
      angle_[0] = angle;
      amp_[0] = amp; 
    }
    else if(data.ID == 0x202)
    {
      rpm_[1] = rpm_data;
      angle_[1] = angle;
      amp_[1] = amp; 
    }
    else if(data.ID == 0x203)
    {
      rpm_[2] = rpm_data;
      angle_[2] = angle;
      amp_[2] = amp; 
    }
    else if(data.ID == 0x204)
    {
      rpm_[3] = rpm_data;
      angle_[3] = angle;
      amp_[3] = amp; 
    }
    else if(data.ID == 0x205)
    {
      rpm_[4] = rpm_data;
      angle_[4] = angle;
      amp_[4] = amp; 
    }
    else if(data.ID == 0x206)
    {
      rpm_[5] = rpm_data;
      angle_[5] = angle;
      amp_[5] = amp; 
    }
    else if(data.ID == 0x207)
    {
      rpm_[6] = rpm_data;
      angle_[6] = angle;
      amp_[6] = amp; 
    }
    else if(data.ID == 0x208)
    {
      rpm_[7] = rpm_data;
      angle_[7] = angle;
      amp_[7] = amp; 
    }
  }

  int RoboMaster::getRPM(uint8_t id)
  {
    return rpm_[id-1];
  }

  int RoboMaster::getAngle(uint8_t id)
  {
    return angle_[id-1];
  }
}

