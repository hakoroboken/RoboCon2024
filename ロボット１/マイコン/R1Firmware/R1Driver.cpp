#include "R1Driver.h"


using namespace r1_firmware;

void R1Driver::setup()
{
  Serial.begin(115200);
  packet = Packet();
  delay(1000);

  motor_handler.setup();
  ethernet_handler.setup();
}

void R1Driver::connection()
{
  ethernet_handler.receive();

  packet = ethernet_handler.create_packet();
}

void R1Driver::control()
{
  bool sw1 = motor_handler.limit_switch_1();
  bool sw2 = motor_handler.limit_switch_2();

  motor_handler.control_front(packet.front);
  motor_handler.control_rear_left(packet.rear_left);
  motor_handler.control_rear_right(packet.rear_right);
  motor_handler.control_valve(packet.valve_signal, sw1, sw2);

  String sw1_str(sw1);
  String sw2_str(sw2);

  String send_packet = sw1_str + "," + sw2_str;

  ethernet_handler.send(send_packet);
}