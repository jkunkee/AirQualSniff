
#define JET_TEST
//#define JET_TEST_TRACING
#define JET_EVT_HUB_TEMPORAL

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
    Serial.println(String("evt::HubTest returned ")+jet::evt::HubTest());
    Serial.println(String("UInt32Test returned ")+jet::UInt32Test());
    tests_have_run = true;
  }
}
