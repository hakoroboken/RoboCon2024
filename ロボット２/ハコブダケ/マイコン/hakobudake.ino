#include "hakorobo2024.h"

#define AMP 16000
#define M 10000

using namespace hakorobo2024;

RoboMaster r;
RoboMaster::Cmd cmd;
RoboMaster::FeedBack fb;
RoboMasterControl rmc(M2006RPM, M2006CURRENT);
MCP2515 can;
I2C_Receiver i2c;

byte len = 0;

void setup()
{
  Serial.begin(115200);
  can.initialize();
  i2c.initialize(8);
  r.setType(1, RoboMaster::Type::M2006);
  r.setType(2, RoboMaster::Type::M2006);
  r.setType(3, RoboMaster::Type::M3508);

  cmd = RoboMaster::Cmd::Cmd();
  fb = RoboMaster::FeedBack::FeedBack();
}

void loop()
{
  Packet p = parse_str(i2c.get_data());

  // Serial.println(i2c.get_data());

  int16_t l_out = (int)AMP * p.value_1;
  int16_t r_out = (int)AMP * p.value_2;
  int16_t m_out = (int)M * p.value_3;

  r.setCurrent(1, l_out);
  r.setCurrent(2, r_out);
  r.setCurrent(3, m_out);

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