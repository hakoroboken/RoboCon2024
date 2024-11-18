#ifndef PACKET_HPP_
#define PACKET_HPP_

#include <Arduino.h>

namespace r1_firmware
{
  struct Packet
  {
    int front;
    int rear_left;
    int rear_right;
    int valve_signal;

    Packet():
    front(0),rear_left(0),rear_right(0),valve_signal(0)
    {

    }
  };

  Packet parse_str(String read);
}

#endif