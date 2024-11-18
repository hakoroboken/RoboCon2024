#ifndef UDPCONNECTION_HPP_
#define UDPCONNECTION_HPP_

#include <Ethernet.h>
#include <EthernetUdp.h>

#include "Packet.h"

#define PACKET_BUFFER_SIZE 64

namespace r1_firmware
{
  class EthernetHandler
  {
    public:
    void setup();
    void receive();
    void send(String packet);
    Packet create_packet();

    private:
    EthernetUDP udp;
    char buf[PACKET_BUFFER_SIZE];
    Packet p;
  };
}

#endif