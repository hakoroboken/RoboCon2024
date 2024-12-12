#include <WiFi.h>
#include <WiFiUdp.h>

const String wifi_name = "robo2Server";
const String wifi_pass = "hakoroboken";
const int wifi_channel = 6;
const int local_port = 64202;
const IPAddress ip(192,168,128,181);
const IPAddress geteway(192,168,128,181);
const IPAddress subnet(255,255,255,0);
WiFiUDP udp;

char buf[256];
int i = 0;
unsigned long time_hold = 0;

int packetSize;
int k = 0;
bool flag = true;

void setup() 
{
  pinMode(T4, OUTPUT);
  pinMode(T5, OUTPUT);
  pinMode(T6, OUTPUT);
  pinMode(T7, OUTPUT);
  pinMode(T8, OUTPUT);
  pinMode(T9, OUTPUT);

  digitalWrite(T4, LOW);
  digitalWrite(T5, LOW);
  digitalWrite(T6, LOW);
  digitalWrite(T7, LOW);
  digitalWrite(T8, LOW);
  digitalWrite(T9, LOW);

  Serial.begin(115200);
  pinMode(2,OUTPUT);

  while (flag == true) 
  {
    WiFi.config(ip, geteway, subnet);
    WiFi.begin(wifi_name, wifi_pass, wifi_channel);
    Serial.print("WiFi connecting");

    while (WiFi.status() != WL_CONNECTED) 
    {
      delay(500);
      Serial.print(".");      
    }
    if (WiFi.status() == WL_CONNECTED) 
    {
      udp.begin(local_port);
      Serial.println("WiFi connected");
      digitalWrite(2, HIGH);
      flag = false;
    } 
  }
}

void loop() 
{
  packetSize = udp.parsePacket();
  if(packetSize)
  {
    time_hold = millis();

    udp.read(buf, 256);

    if(buf[i] == 'e')
    {
      String data(buf);

      Serial.println(data);

      int first_com = data.indexOf(',');
      String l_str = data.substring(0, first_com);
      String non_first = data.substring(first_com+1);

      int second_com = non_first.indexOf(',');
      String r_str = non_first.substring(0, second_com);
      String non_second = non_first.substring(second_com+1);

      int third_com = non_second.indexOf(',');
      String m1_str = non_second.substring(0, third_com);
      String non_third = non_second.substring(third_com+1);

      int fourth_com = non_third.indexOf('e');
      String m2_str = non_third.substring(0, fourth_com);

      int l_hold = l_str.toInt();
      int r_hold = r_str.toInt();
      int m1_hold = m1_str.toInt();
      int m2_hold = m2_str.toInt();

      int l = (l_hold - 20);
      int r = (r_hold - 20);
      int m1 = (m1_hold - 20);
      int m2 = (m2_hold - 20);

      // Serial.print(l);
      // Serial.print(",");
      // Serial.print(r);
      // Serial.print(",");
      // Serial.print(m1);
      // Serial.print(",");
      // Serial.println(m2);

      if(l > 0)
      {
        digitalWrite(T4, HIGH);
        digitalWrite(T5, LOW);
      }
      else if(l < 0)
      {
        digitalWrite(T4, LOW);
        digitalWrite(T5, HIGH);
      }
      else
      {
        digitalWrite(T4, LOW);
        digitalWrite(T5, LOW);
      }

      if(r > 0)
      {
        digitalWrite(T6, HIGH);
        digitalWrite(T7, LOW);
      }
      else if(r < 0)
      {
        digitalWrite(T6, LOW);
        digitalWrite(T7, HIGH);
      }
      else
      {
        digitalWrite(T6, LOW);
        digitalWrite(T7, LOW);
      }

      if(m1 > 0)
      {
        digitalWrite(T8, HIGH);
        digitalWrite(T9, LOW);
      }
      else if(m1 < 0)
      {
        digitalWrite(T8, LOW);
        digitalWrite(T9, HIGH);
      }
      else
      {
        digitalWrite(T8, LOW);
        digitalWrite(T9, LOW);
      }
      
      for(int j = 0; j <= 256; j++)
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
    digitalWrite(T4, LOW);
    digitalWrite(T5, LOW);
    digitalWrite(T6, LOW);
    digitalWrite(T7, LOW);
    digitalWrite(T8, LOW);
    digitalWrite(T9, LOW);
    Serial.println("error");
  }
}