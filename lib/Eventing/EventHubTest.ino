#define EVENTHUB_DEBUG
#include "Eventing.h"

//Test scaffolding

void printme(Eventing::Event* event, Eventing::EventTrigger *out) {
    Serial.printlnf("printing %s", event->name);
    for (int idx_e = 0; idx_e < event->triggers.count; idx_e++) {
        Eventing::EventTrigger *trigger = event->triggers.list[idx_e];
        if (trigger->data_ready) {
            Serial.printlnf("Event \"%s\" %0.2f/%u/%d/%x", trigger->source_event->name, trigger->data.fl, trigger->data.uin16, trigger->data.in16, trigger->data.uin16);
        }
    }
}

using namespace Eventing;

EventHub hub;

// Thing A happens very periodically
// Thing B happens in loop() sometimes
// Thing C is a transform of A
// Thing D happens very periodically and produces multiple data at a time
// Thing D1 is one of those things
// Thing D2 is one of those things
// Thing E depends on B and A->C
// Thing F depends on A or B or D

void ThingA_Action(Event *e, EventTrigger *out) {

}
Event ThingA(&ThingA_Action, "ThingA", EventTriggerType::TRIGGER_TEMPORAL, 1000);

Event ThingB(NULL, "ThingB", TRIGGER_MANUAL);
Event ThingC(NULL, "ThingC", TRIGGER_ON_ALL);
Event ThingD(NULL, "ThingD", TRIGGER_TEMPORAL, 1000);
Event ThingD1(NULL, "ThingD1", TRIGGER_MANUAL); // currently you need pretend events like these to broadcast multiple simultanously
Event ThingD2(NULL, "ThingD2", TRIGGER_MANUAL);
Event ThingE(printme, "ThingE", TRIGGER_ON_ALL);
Event ThingF(printme, "ThingF", TRIGGER_ON_ANY);

void setup() {
    hub.Add(&ThingA);
    hub.Add(&ThingB);
    hub.Add(&ThingC);
    ThingC.AddTrigger(&ThingA);
    hub.Add(&ThingD);
    hub.Add(&ThingE);
    ThingE.AddTrigger(&ThingB);
    ThingE.AddTrigger(&ThingC);
    ThingE.AddTrigger(&ThingD1);
    hub.Add(&ThingF);
    ThingF.AddTrigger(&ThingA);
    ThingF.AddTrigger(&ThingB);
    ThingF.AddTrigger(&ThingD2);
    hub.begin();
}

void loop() {
    hub.update();
}
