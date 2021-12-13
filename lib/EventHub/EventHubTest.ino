#define EVENTHUB_DEBUG
#include "EventHub.h"

static bool foo(EventHub::Event* event);
static bool foo(EventHub::Event* event) {
    Serial.printlnf("fired foo");
    for (size_t input_idx = 0; input_idx < event->triggers_count; input_idx++) {
        Serial.printlnf("  Rx'd #%d %f", input_idx, event->triggers[input_idx].data.fl);
    }
    return true;
}

static EventHub::EventHub event_hub_actual = {
    .events_count = 3,
    .events = {
        {
            .action = NULL,
            .triggers_count = 0,
            .triggers = {},
        },
        {
            .action = NULL,
            .triggers_count = 0,
            .triggers = {},
        },
        {
            .action = &foo,
            .triggers_count = 2,
            .triggers = {
                {
                    .source_event = &event_hub_actual.events[0],
                },
                {
                    .source_event = &event_hub_actual.events[1],
                },
            },
        },
    },
};
