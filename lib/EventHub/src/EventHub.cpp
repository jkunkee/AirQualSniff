
#define EVENTHUB_DEBUG

#include <Arduino.h>
#include "EventHub.h"

#ifdef EVENTHUB_DEBUG
#define EVH_LOG(...) Serial.printlnf(__VA_ARGS__)
#else
#define EVH_LOG(...)
#endif

namespace EventHub {

void ResetEventTriggers(Event& event) {
    for (size_t trigger_idx = 0; trigger_idx < event.triggers_count; trigger_idx++) {
        event.triggers[trigger_idx].data_ready = false;
        event.triggers[trigger_idx].data.fl = -1;
    }
}

bool FireEvent(EventHub* hub, Event* event_to_emit, EventData* data_to_emit) {
    if (hub == NULL || event_to_emit == NULL || data_to_emit == NULL) {
        EVH_LOG("bad arg ptr %p, %p, %p", hub, event_to_emit, data_to_emit);
        return false;
    }
    for (uint event_idx = 0; event_idx < hub->events_count; event_idx++) {
        Event* event_to_check = &(hub->events[event_idx]);
        bool checked_event_triggered = event_to_check->triggers_count > 0;
        for (uint trigger_idx = 0; trigger_idx < event_to_check->triggers_count; trigger_idx++) {
            EventTrigger* trigger = &(event_to_check->triggers[trigger_idx]);
            if (event_to_emit == trigger->source_event) {
                trigger->data_ready = true;
                trigger->data = *data_to_emit;
            }
            checked_event_triggered &= trigger->data_ready;
        }
        if (checked_event_triggered) {
            event_to_check->action(event_to_check);
            ResetEventTriggers(*event_to_check);
        }
    }
    return true;
}

void DumpStateOnSerial(EventHub* hub) {
    Serial.printlnf("Hub %p, %u events", hub, hub->events_count);
    for (size_t event_idx = 0; event_idx < hub->events_count; event_idx++) {
        Event& event = hub->events[event_idx];
        Serial.printlnf("  Event %p, %u triggers", &hub->events[event_idx], event.triggers_count);
        for (size_t trigger_idx = 0; trigger_idx < event.triggers_count; trigger_idx++) {
            EventTrigger& trigger = event.triggers[trigger_idx];
            Serial.printlnf("    Trigger %d, event: %p, data ready: %d, data: %f", trigger_idx, trigger.source_event, trigger.data_ready, trigger.data.fl);
        }
    }
}

} // namespace EventHub
