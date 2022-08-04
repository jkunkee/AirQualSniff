
#pragma once

#include <Arduino.h>

//#define DC_TEST

typedef void (*DeltaClockAction)(void);

typedef struct _DeltaClockEntry {
    DeltaClockAction action;
    unsigned long interval;
    boolean repeating;
    unsigned long remaining;
    struct _DeltaClockEntry* next;
} DeltaClockEntry;

class DeltaClock {
public:
    DeltaClock() : head(NULL), lastUpdate(0) {}
    bool begin() {
#ifdef DC_TEST
        Serial.begin();
        printToSerial();
#endif
        return true;
    }
    // mark the passage of time, call expired events, and requeue repeating events
    void update();
    // insert a new entry into the delta clock
    // This follows a FIFO pattern: if two events are queued that expire at the same time,
    // the first event happens first.
    // If an entry expires multiple times, it will run the expected number of times all at once.
    void insert(DeltaClockEntry* entry);
    // dequeue all events
    void clear() {
        head = NULL;
    }
private:
    DeltaClockEntry* head;
    unsigned long lastUpdate;
#ifdef DC_TEST
    void printToSerial();
#endif
};
