
#define JET_TEST

#include "jet.h"

bool tests_have_run = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!tests_have_run) {
    Serial.println(jet::PointerListTest());
    tests_have_run = true;
  }
}
