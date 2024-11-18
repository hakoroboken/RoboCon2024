#include "UdpConnection.h"

using namespace r1_firmware;

void EthernetHandler::setup()
{
  p = Packet();
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  
  Ethernet.init(17);
  Serial.println("[EthernetHandler] Initialized Ethernet.");

  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x05 };
  IPAddress default_ip(192, 168, 11, 2);
  
  Ethernet.begin(mac, default_ip);

  Serial.print("[EthernetHandler] Start Ethernet on Address : ");
  Serial.println(Ethernet.localIP());

  while(udp.begin(64205) == 0)
  {
    Serial.print(".");
  }
  Serial.println("[EthernetHandler] Start UDP on port : 64205");

  Serial.println("[EthernetHandler] EthernetHandler OK!!!!!!!");

  digitalWrite(LED_BUILTIN, HIGH);
}

void EthernetHandler::receive()
{
  for(int i = 0; i < PACKET_BUFFER_SIZE; i++)
  {
    buf[i] = 0;
  }
  int packet_size = udp.parsePacket();

  if(packet_size > 0)
  {
    udp.read(buf, PACKET_BUFFER_SIZE);

    String get_str(buf);

    p = parse_str(get_str);
  }
}

void EthernetHandler::send(String packet)
{
  IPAddress destination(192, 168, 11, 3);
  auto port = 64201;

  udp.beginPacket(destination, port);
  udp.write(packet.c_str());
  udp.endPacket();
}

Packet EthernetHandler::create_packet()
{
  return p;
}