
#pragma once

#include <Arduino.h>

typedef void (*DeltaClockAction)(void);

typedef struct _DeltaClockEntry {
    DeltaClockAction action;
    unsigned long interval;
    unsigned long remaining;
    boolean repeating;
    struct _DeltaClockEntry* next;
} DeltaClockEntry;

class DeltaClock {
public:
    DeltaClock();
    void begin();
    void update();
    void insert(DeltaClockEntry* entry);
private:
    DeltaClockEntry* head;
    unsigned long lastUpdate;
};
