
#pragma once

namespace EventHub {

typedef struct _Event Event;

typedef union _EventData {
    float fl;
    int16_t in16;
    uint16_t uin16;
} EventData;

typedef struct _EventTrigger {
    Event* source_event;
    EventData data;
    bool data_ready;
} EventTrigger;

typedef bool (*EventAction)(Event* event);

typedef struct _Event {
    EventAction action;
    size_t triggers_count;
    EventTrigger triggers[2];
} Event;

typedef struct _EventHub {
    size_t events_count;
    Event events[3];
} EventHub;

bool FireEvent(EventHub* event_hub, Event* event, EventData* data);

#ifdef EVENTHUB_DEBUG
void DumpStateOnSerial(EventHub* hub);
#endif

} // namespace EventHub
