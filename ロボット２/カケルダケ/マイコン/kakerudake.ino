#include "hakorobo2024.h"

#define AMP 4000
#define M 10000

using namespace hakorobo2024;

RoboMaster r;
RoboMaster::Cmd cmd;
RoboMaster::FeedBack fb;
MCP2515 can;
I2C_Receiver i2c;
Wheel2 wheel;

byte len = 0;

void setup()
{
  Serial.begin(115200);
  can.initialize();
  i2c.initialize(8);
  r.setType(1, RoboMaster::Type::M2006);
  r.setType(2, RoboMaster::Type::M2006);
  r.setType(3, RoboMaster::Type::M2006);
  r.setType(4, RoboMaster::Type::M2006);

  cmd = RoboMaster::Cmd::Cmd();
  fb = RoboMaster::FeedBack::FeedBack();
}

void loop()
{
  Packet p = parse_str(i2c.get_data());

  Serial.println(i2c.get_data());

  float y = p.value_1;
  float ro = p.value_2;

  wheel.set_vec(y, ro);  
  int16_t l_out = (int)AMP * wheel.get_l();
  int16_t r_out = (int)AMP * wheel.get_r();
  int16_t m1_out = (int)M * p.value_3;
  int16_t m2_out = (int)M * p.value_4;

  // Serial.print(m1_out);
  // Serial.print(",");
  // Serial.println(m2_out);

  r.setCurrent(1, l_out);
  r.setCurrent(2, r_out);
  r.setCurrent(3, m1_out);
  r.setCurrent(4, m2_out);

  cmd = r.createCommand();
  can.send(0x200, cmd.buf_1);

  can.recv(&fb.ID, len, fb.buf);
  r.setFeedBack(fb);
  can.recv(&fb.ID, len, fb.buf);
  r.setFeedBack(fb);
  can.recv(&fb.ID, len, fb.buf);
  r.setFeedBack(fb);
  can.recv(&fb.ID, len, fb.buf);
  r.setFeedBack(fb);
  can.recv(&fb.ID, len, fb.buf);
  r.setFeedBack(fb);
  can.recv(&fb.ID, len, fb.buf);
  r.setFeedBack(fb);
}