#include "I2C.h"

int index = 0;
char buf[256];

namespace hakorobo2024
{
  void I2C_Receiver::initialize(uint8_t id)
  {
    Wire.begin(id);
    Wire.onReceive(i2c_receive);
  }

  String I2C_Receiver::get_data()
  {
    String data(buf);

    if(data == "")
    {
      data = "20,20,20,20e";
    }

    return data;
  }

  void I2C_Transporter::initialize()
  {
    Wire.begin();

    PreMillis = millis();
  }

  void I2C_Transporter::transport(uint8_t id, String msg)
  {
    if(millis()- PreMillis > 5)
    {
      Wire.beginTransmission(id);
      Wire.write(msg.c_str());
      Wire.endTransmission();

      PreMillis = millis();
    }
  }
}

void i2c_receive(int howmany)
{
  for(int i = 0; i < 256; i++)buf[i] = 0;

  while (Wire.available()) {
    buf[index] = Wire.read();
    index++;
  }

  index = 0;
}