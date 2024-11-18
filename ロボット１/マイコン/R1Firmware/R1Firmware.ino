#include "R1Driver.h"

using namespace r1_firmware;

R1Driver driver;

void setup() {
  driver.setup();
}

void loop() {
  driver.connection();
  driver.control();
}
