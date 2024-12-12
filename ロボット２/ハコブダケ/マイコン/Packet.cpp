#include "Packet.h"

namespace hakorobo2024
{
  Packet parse_str(String read)
  {
    int start_index = 0;
    int end_index = read.indexOf('e');
    String data = read.substring(start_index, end_index);

    int first_com = data.indexOf(',');
    String str_1 = data.substring(0, first_com);

    String without_1 = data.substring(first_com+1);

    int second_com = without_1.indexOf(',');

    String str_2 = without_1.substring(0, second_com);
    String without_2 = without_1.substring(second_com+1);

    int third_com = without_2.indexOf(',');

    String str_3 = without_2.substring(0, third_com);
    String without_3 = without_2.substring(third_com+1);

    float v1 = str_1.toFloat();
    float v2 = str_2.toFloat();
    float v3 = str_3.toFloat();
    float v4 = without_3.toFloat();

    Packet new_packet = Packet();
    new_packet.value_1 = (v1 - 20) / 10.0;
    new_packet.value_2 = (v2 - 20) / 10.0;
    new_packet.value_3 = (v3 - 20) / 10.0;
    new_packet.value_4 = (v4 - 20) / 10.0;

    return new_packet;
  }
}