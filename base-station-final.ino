#include "lora.h"

Timer t(MICROS);
lora coms(Serial7, t);

void setup() {
  // put your setup code here, to run once:
  t.start();
}

void loop() {
  // put your main code here, to run repeatedly:
  coms.recvWithChecksum();
  coms.showNewData();
}
