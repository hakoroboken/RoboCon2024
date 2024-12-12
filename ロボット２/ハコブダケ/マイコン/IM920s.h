#ifndef IM920S_HPP_
#define IM920S_HPP_

#include <Arduino.h>

#define PACKET_BUFFER_SIZE 256

namespace hakorobo2024
{
  /// @brief Serialのデータを読み取るクラス
  /// @note IM920sを使ってデータを受信することができる
  class IM920s
  {
    public:
    void setup();
    void receive();
    String get_packet();

    private:
    char buf[PACKET_BUFFER_SIZE];
    int i;
    unsigned long time_hold;
    String p;
  };
}

#endif