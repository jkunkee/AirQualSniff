/*
 * Project DeltaClock
 * Description:
 * Author:
 * Date:
 */

#include "DeltaClock.h"

DeltaClock deltaClock;

void TestRepOne(void) {
    static int calls = 1;
    Serial.printlnf("%s: %d calls so far", __func__, calls++);
}
void TestRepTwo(void) {
    static int calls = 1;
    Serial.printlnf("%s: %d calls so far", __func__, calls++);
}
DeltaClockEntry One = {
    [] (void) { Serial.println("One"); },
    10,
    false,
};
DeltaClockEntry Two = {
    [] (void) { Serial.println("Two"); },
    10,
    false,
};
DeltaClockEntry Three = {
    [] (void) { Serial.println("Three"); },
    20,
    false,
};
DeltaClockEntry RepOne = {
    &TestRepOne,
    10,
    true,
};
DeltaClockEntry RepTwo = {
    &TestRepTwo,
    13,
    true,
};

// setup() runs once, when the device is first turned on.
void setup() {
  // Put initialization like pinMode and begin functions here.
  Serial.begin(115200);

  deltaClock.begin();
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // The core of your code will likely live here.
  Serial.println("Begin -------------------------");
  deltaClock.update();
  deltaClock.insert(&One);
  deltaClock.insert(&Two);
  deltaClock.insert(&Three);
  delay(21);
  deltaClock.update();
  deltaClock.clear();
  Serial.println("Rep 1: One,One,One,One,One");
  deltaClock.update();
  deltaClock.insert(&RepOne);
  delay(11);
  deltaClock.update();
  delay(11);
  deltaClock.update();
  delay(31);
  deltaClock.update();
  deltaClock.clear();
  Serial.println("Rep 2: One, Two, One, Two, One, Two, One, One, Two");
  deltaClock.update();
  deltaClock.insert(&RepOne);
  deltaClock.insert(&RepTwo);
  delay(4 * RepTwo.interval);
  deltaClock.update();
  deltaClock.clear();
  Serial.println("End   -------------------------");
  delay(6000);
}
