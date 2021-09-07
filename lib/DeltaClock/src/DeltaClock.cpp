
#include "DeltaClock.h"

DeltaClock::DeltaClock() : head(NULL), lastUpdate(0) {}

void DeltaClock::begin() {}

void DeltaClock::update() {
    // Establish amount of time passed, accounting for millis() rollover
    unsigned long now = millis();
    unsigned long passed;
    // if we've hit millis() rollover,
    if (now < lastUpdate) {
        // figure out how much time passed until rollover plus zero until now
        passed = (unsigned long)(-1) - lastUpdate + now;
    } else {
        passed = now - lastUpdate;
    }
    // Update entries
    for (DeltaClockEntry* current = head; current != NULL; current = current->next) {
        // if this event hasn't triggered yet, we're done.
        if (current->remaining > passed) {
            current->remaining -= passed;
            break;
        } else {
            // note the passage of time
            passed -= current->remaining;
            // mark the item's remaining time as passed
            current->remaining = 0;
            // perform the item's action
            current->action();
            // remove the item from the list
            head = current->next;
            current->next = NULL;
            // re-add it if desired
            if (current->repeating) {
                // reschedule
                insert(current);
            }
        }
    }
}

void DeltaClock::insert(DeltaClockEntry* entry) {
    if (entry == NULL || entry->action == NULL || entry->interval == 0) {
        return;
    }
    entry->next = NULL;
    entry->remaining = entry->interval;
    // empty list
    if (head == NULL) {
        head = entry;
        return;
    }
    // insert at beginning
    if (head->remaining >= entry->remaining) {
        // update remaining
        head->remaining -= head->remaining - entry->remaining;
        // insert in data structure
        entry->next = head;
        head = entry;
        return;
    }
    // it goes after head, so subtract off head->remaining
    entry->remaining -= head->remaining;
    for (DeltaClockEntry* current = head; current != NULL; current = current->next) {
        DeltaClockEntry* next = current->next;
        // does entry go between current and next?
        // if next is null, entry becomes next
        if (next == NULL) {
            current->next = entry;
            break;
        }
        // if it fits, insert it
        if (entry->remaining <= next->remaining) {
            next->remaining -= entry->remaining;
            current->next = entry;
            entry->next = next;
            break;
        }
        // it doesn't fit, so move to the next slot
        entry->remaining -= next->remaining;
    }
}
