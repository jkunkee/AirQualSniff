
#pragma once

#include <DeltaClock.h>

namespace Eventing {

template <class T> class PointerList {
public:
    PointerList() {
        list = new T*[5];
        count = 0;
        capacity = 5;
    }
    T** list;
    size_t count;
    size_t capacity;
    bool Add(T* item) {
#ifdef EVENTHUB_DEBUG
        if (count > capacity) {
            // something has gone horribly wrong
            Serial.println("PointList add encountered bad count/capacity values");
            return false;
        }
#endif
        if (count >= capacity) {
            T** new_list = new T*[capacity+2];
            memcpy(new_list, list, capacity*sizeof(T*));
            delete(list);
            list = new_list;
            capacity = capacity+2;
        }
        list[count] = item;
        count++;
        return true; // maybe one day check for OOM and duplicates
    }
};

class Event;

class EventData {
public:
    EventData() : fl(0.0) {}
    union {
        float fl;
        int16_t in16;
        uint16_t uin16;
        void* ptr;
    };
};

class EventTrigger {
public:
    EventTrigger(Event* e) : source_event(e), data_ready(false) {}
    void Reset() {
        data.fl = -NAN;
        data_ready = false;
    }
    Event* source_event;
    EventData data;
    bool data_ready;
};

typedef void (*EventAction)(Event* event, EventTrigger *out);

typedef enum _EventTriggerType {
    TRIGGER_MANUAL,
    TRIGGER_ON_ANY,
    TRIGGER_ON_ALL,
    TRIGGER_TEMPORAL,
} EventTriggerType;

class Event {
public:
    Event(EventAction a, const char* n, EventTriggerType t, unsigned long _interval_ms = 0) : action(a), name(n), type(t), interval_ms(_interval_ms) {}
    bool AddTrigger(Event *trigger) { return triggers.Add(new EventTrigger(trigger)); }
    bool ProcessTrigger(Event* src_event, EventData* data, EventTrigger* out) {
        bool trigger_found = false;
        for (size_t idx = 0; idx < triggers.count; idx++) {
            if (triggers.list[idx]->source_event == src_event) {
                triggers.list[idx]->data = *data;
                triggers.list[idx]->data_ready = true;
                trigger_found = true;
                break;
            }
        }
        if (triggers.count > 0) {
            bool is_triggered = false;
            if (type == TRIGGER_ON_ALL) {
                is_triggered = true;
                for (size_t idx = 0; idx < triggers.count; idx++) {
                    is_triggered = is_triggered && triggers.list[idx]->data_ready;
                }
            } else if (type == TRIGGER_ON_ANY) {
                is_triggered = false;
                for (size_t idx = 0; idx < triggers.count; idx++) {
                    is_triggered = is_triggered || triggers.list[idx]->data_ready;
                }
            }
            if (is_triggered) {
                action(this, out);
                ResetTriggers();
            }
        }
        return trigger_found;
    }
    void ResetTriggers() {
        for (size_t idx = 0; idx < triggers.count; idx++) {
            triggers.list[idx]->Reset();
        }
    }

    EventAction action;
    const char* name;
    EventTriggerType type;
    PointerList<EventTrigger> triggers;
    unsigned long interval_ms;
};

class EventHub {
private:
    DeltaClock delta_clock;
    PointerList<DeltaClockEntry> delta_clock_entries;
public:
    EventHub() {}
    bool Add(Event* event) {
        if (event->type == TRIGGER_TEMPORAL) {
            DeltaClockEntry *entry = new DeltaClockEntry;
            *entry = { 0 };
            entry->interval = event->interval_ms;
            entry->repeating = true;
            entry->action = ?;
            delta_clock_entries.Add(entry);
        }
        return events.Add(event);
    }
    bool Fire(Event* event, EventData* data) {
        bool trigger_found = false;
        for (size_t e_idx = 0; e_idx < events.count; e_idx++) {
            EventTrigger response(NULL);
            trigger_found = events.list[e_idx]->ProcessTrigger(event, data, &response) || trigger_found;
            if (response.data_ready) {
                Fire(events.list[e_idx], &response.data);
            }
        }
        return trigger_found;
    }
    void begin() {
        delta_clock.begin();
    }
    void update() {
        delta_clock.update();
    }
    PointerList<Event> events;
#ifdef EVENTHUB_DEBUG
    void DumpStateOnSerial() {
        Serial.printlnf("Hub %p, %u events", this, events.count);
        for (size_t event_idx = 0; event_idx < events.count; event_idx++) {
            Event& event = *events.list[event_idx];
            Serial.printlnf(
                "  Event %p \"%s\", %u triggers, type %d",
                events.list[event_idx],
                event.name ? event.name : "",
                event.triggers.count,
                event.type);
            for (size_t trigger_idx = 0; trigger_idx < event.triggers.count; trigger_idx++) {
                EventTrigger& trigger = *event.triggers.list[trigger_idx];
                Serial.printlnf("    Trigger %d, event: %p, data ready: %d, data: %f/%u/%d", trigger_idx, trigger.source_event, trigger.data_ready, trigger.data.fl, trigger.data.uin16, trigger.data.in16);
            }
        }
    }
#endif
};

} // namespace Eventing
