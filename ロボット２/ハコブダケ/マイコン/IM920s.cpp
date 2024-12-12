#include "IM920s.h"

using namespace hakorobo2024;

void IM920s::setup()
{
  Serial.begin(115200);

  i = 0;
  time_hold = 0;
}

void IM920s::receive()
{
  if(Serial.available())
  {
    buf[i] = Serial.read();
    time_hold = millis();
    if (buf[i] == '\n')
    {
      if(buf[0] != '0' || i != 24)
      {

      }
      else
      {
        String get_str(buf);

        p = get_str.substring(11);
      }
      
      for(int j = 0; j <= i; j++)
      {
        buf[j] = 0;
      }
      i = 0;
    }
    else
    {
        i++;
    }
  }
  else if((millis() - time_hold) > 100) 
  {
    time_hold = millis();
    p = "";
  }
}

String IM920s::get_packet()
{
  return p;
}