
#define JET_TEST

#include "..\..\src\jet.h"

bool tests_have_run = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!tests_have_run) {
    Serial.println(String("PointerListTest returned ")+jet::PointerListTest());
    Serial.println(String("evt::DeltaClockTest returned ")+jet::evt::DeltaClockTest());
    tests_have_run = true;
  }
}
