#ifndef PACKET_HPP_
#define PACKET_HPP_

#include <Arduino.h>

namespace hakorobo2024
{
  struct Packet
  {
    float value_1;
    float value_2;
    float value_3;
    float value_4;

    Packet():
    value_1(0.0),value_2(0.0),value_3(0.0),value_4(0.0)
    {

    }
  };

  Packet parse_str(String read);
}

#endif