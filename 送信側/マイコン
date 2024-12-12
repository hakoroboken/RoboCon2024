#include <SoftwareSerial.h>

SoftwareSerial IM920Serial(8, 9);

char buf[256];
int i = 0;

void setup() 
{
  Serial.begin(115200);
  IM920Serial.begin(115200);

  pinMode(10, INPUT);
}

void loop() {
  if (Serial.available()) 
  {
    buf[i] = Serial.read();
    if (buf[i] == 'e') {
      if (digitalRead(10) == LOW)
      {
        IM920Serial.print("TXDU0003,");
        IM920Serial.println(buf);
        // Serial.println(buf);
      }
      for (int j = 0; j <= i; j++) 
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
  if (i >= 256) 
  {
    i = 0;
  }
}
