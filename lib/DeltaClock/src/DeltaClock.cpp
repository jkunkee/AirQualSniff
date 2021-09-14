
#include "DeltaClock.h"

DeltaClock::DeltaClock() : head(NULL), lastUpdate(0) {}

void DeltaClock::begin() {
#if DC_TEST
    Serial.begin();
    printToSerial();
#endif
}

void DeltaClock::update() {
    // Establish amount of time passed, accounting for millis() rollover
    unsigned long now = millis();
    unsigned long delta;
    // if we've hit millis() rollover,
    if (now < lastUpdate) {
        // figure out how much time passed until rollover plus zero until now
        delta = (unsigned long)(-1) - lastUpdate + now;
    } else {
        delta = now - lastUpdate;
    }
#if DC_TEST
    Serial.printf("Update delta: %d... ", delta);
#endif
    // Update entries
    while (head != NULL) {
        if (head->remaining > delta) {
            head->remaining -= delta;
            break;
        } else {
            // note the passage of time
            delta -= head->remaining;
            head->remaining = 0;
            // execute
            head->action();
            // remove the item from the list
            DeltaClockEntry* old = head;
            head = head->next;
            old->next = NULL;
            // requeue if repeating
            if (old->repeating) {
                insert(old);
            }
        }
    }
    lastUpdate = now;
#if DC_TEST
    Serial.printf("Update complete. ");
    printToSerial();
#endif
}

void DeltaClock::insert(DeltaClockEntry* entry) {
    if (entry == NULL || entry->action == NULL || entry->interval == 0) {
#if DC_TEST
    Serial.printf("Insertion failed, bad argument(s). ");
#endif
        return;
    }
    DeltaClockEntry* prev = NULL;
    DeltaClockEntry* next = head;
    // prep for insertion
    entry->next = NULL;
    entry->remaining = entry->interval;
    // empty list
    if (head == NULL) {
        head = entry;
        goto cleanup;
    }
    // find insertion point; start at head
    while (true) {
        // if this is the insertion point, insert
        if (prev == NULL && entry->remaining < next->remaining) {
            // beginning
            entry->next = next;
            head = entry;
            next->remaining -= entry->remaining;
            break;
        } else if (next == NULL) {
            // end
            prev->next = entry;
            break;
        } else if (entry->remaining < next->remaining) {
            // middle
            prev->next = entry;
            entry->next = next;
            next->remaining -= entry->remaining;
            break;
        }
        // move to next entry
        entry->remaining -= next->remaining;
        prev = next;
        next = next->next;
    }

cleanup:
#if DC_TEST
    Serial.printf("Insertion complete. ");
    printToSerial();
#endif
    return;
}

void DeltaClock::clear() {
    head = NULL;
}

#if DC_TEST
void printEntry(DeltaClockEntry* entry) {
    Serial.printf("this: %p ", entry);
    if (entry != NULL) {
        Serial.printlnf("action: %p interval: %ld repeating: %d remaining: %ld next: %p",
            entry->action,
            entry->interval,
            entry->repeating,
            entry->remaining,
            entry->next);
    }
}

void DeltaClock::printToSerial() {
    Serial.printlnf("head: %p lastUpdate: %ld", head, lastUpdate);
    for (DeltaClockEntry* entry = head; entry != NULL; entry = entry->next) {
        printEntry(entry);
    }
}
#endif // DC_TEST
