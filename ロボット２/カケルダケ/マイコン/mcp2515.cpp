#include "mcp2515.h"

MCP_CAN can0(10);

namespace hakorobo2024
{
  void MCP2515::initialize()
  {
    if (can0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
    {
      Serial.println("CAN0: Init OK!");
      can0.setMode(MCP_NORMAL);
    }
    else
    {
      Serial.println("CAN0: Init Fail!");
    }

    Pre_millis = millis();
  }

  void MCP2515::send(unsigned long int dest, byte *buf)
  {
    if(millis() - Pre_millis > 50)
    {
      auto b = can0.sendMsgBuf(dest, 0, 8, buf);
      if(b == CAN_OK)
      {
        // Serial.println("CAN_SEND_OK");
      }
      Pre_millis = millis();
    }
  }

  void MCP2515::recv(unsigned long *id, byte *len, char *buf)
  {
    if(can0.checkReceive() == CAN_MSGAVAIL)
    {
      can0.readMsgBuf(id, len, buf);
    }
  }
}