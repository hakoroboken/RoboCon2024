#ifndef R1_DRIVER_HPP_
#define R1_DRIVER_HPP_

#include "MotorControl.h"
#include "UdpConnection.h"

#include "Packet.h"

namespace r1_firmware
{
  class R1Driver
  {
    public:
    void setup();
    void connection();
    void control();

    private:
    EthernetHandler ethernet_handler;
    MotorHandler motor_handler;

    Packet packet;
  };
}

#endif