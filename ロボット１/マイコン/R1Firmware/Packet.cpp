#include "Packet.h"

namespace r1_firmware
{
  Packet parse_str(String read)
  {
    int start_index = 0;
    int end_index = read.indexOf('e');
    String data = read.substring(start_index, end_index);

    int first_div_index = data.indexOf(',');
    String str_1 = data.substring(0, first_div_index);

    String without_1 = data.substring(first_div_index+1);

    int second_com = without_1.indexOf(',');

    String str_2 = without_1.substring(0, second_com);
    String without_2 = without_1.substring(second_com+1);

    int third_com = without_2.indexOf(',');

    String str_3 = without_2.substring(0, third_com);
    String without_3 = without_2.substring(third_com+1);

    int v1 = str_1.toInt();
    int v2 = str_2.toInt();
    int v3 = str_3.toInt();
    int v4 = without_3.toInt();

    Packet new_packet = Packet();
    new_packet.front = v1 - 300;
    new_packet.rear_left = v2 - 300;
    new_packet.rear_right = v3 - 300;
    new_packet.valve_signal = v4 - 300;

    // Serial.printf("%d,%d,%d,%d\n", new_packet.front, new_packet.rear_left, new_packet.rear_right, new_packet.valve_signal);

    return new_packet;
  }
}