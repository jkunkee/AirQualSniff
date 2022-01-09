#define EVENTHUB_DEBUG
#include "EventHub.h"

//Test scaffolding

bool printme(Eventing::Event* event) {
    Serial.printlnf("printing %s", event->name);
    return true;
}

Eventing::Event one(&printme, "one", Eventing::TRIGGER_NONE);
Eventing::Event two(&printme, "two", Eventing::TRIGGER_NONE);
Eventing::Event three(&printme, "three", Eventing::TRIGGER_ON_ANY);
Eventing::EventHub test_hub;

void TimeFire() {
    Serial.println("TimeFire");
    Eventing::EventData d;
    d.fl = 2.0;
    test_hub.Fire(&two, &d);
}
DeltaClockEntry TimeFireEntry = {
    .action = &TimeFire,
    .interval = 2000,
    .repeating = true,
};


    test_hub.Add(&one);
    test_hub.Add(&two);
    test_hub.Add(&three);
    three.AddTrigger(&one);
    infrastructure::deltaClock.insert(&TimeFireEntry);

    //test_hub.DumpStateOnSerial();
    Eventing::EventData d;
    d.fl = 0.1;
    test_hub.Fire(&one, &d);






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
