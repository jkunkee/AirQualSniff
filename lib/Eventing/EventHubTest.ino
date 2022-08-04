
#include <Arduino.h>

#define EVENTHUB_DEBUG
#define EVENTHUB_TEMPORAL
#include "Eventing.h"
using namespace Eventing;

#include "DeltaClock.h"

#include <Arduino_DebugUtils.h>
Arduino_DebugUtils d;

// Thing A happens very periodically
// Thing B happens in loop() sometimes
// Thing C is a transform of A
// Thing D happens very periodically and produces multiple data at a time
// Thing D1 is one of those things
// Thing D2 is one of those things
// Thing E depends on B and A->C
// Thing F depends on A or B or D

//bool (*EventHandlerFunc)(PointerList<EventTrigger>& triggers, EventData& out);

bool GenA(PointerList<EventTrigger>& triggers, EventData& out) {
  out.in16 = 3;
  d.print(DBG_INFO, "GenA");
  return true;
}

bool AtoC(PointerList<EventTrigger>& triggers, EventData& out) {
  d.print(DBG_INFO, "AtoC fired");
  for (int trigIdx = 0; trigIdx < triggers.count; trigIdx++) {
    d.print(DBG_INFO, "AtoC checking %d", trigIdx);
    EventTrigger* trig = triggers.list[trigIdx];
    if (trig->data_ready && trig->event_id.equalsIgnoreCase(String("A"))) {
      out.in16 = trig->data.in16 + 2;
      d.print(DBG_INFO, "AtoC: %d+%d=%d", trig->data.in16, 2, out.in16);
      return true;
    }
  }
  d.print(DBG_INFO, "AtoC failed!");
  return false;
}

typedef struct _D_DATA {
  uint8_t byte1;
  uint8_t byte2;
} D_DATA;

D_DATA DataForD;

bool GenD(PointerList<EventTrigger>& triggers, EventData& out) {
  d.print(DBG_INFO, "GenD");
  DataForD.byte1 = 0x0f;
  DataForD.byte1 = 0xf0;
  out.ptr = &DataForD;
  return true;
}

bool GenD1(PointerList<EventTrigger>& triggers, EventData& out) {
  d.print(DBG_INFO, "GenD1");
  D_DATA* data = triggers.list[0]->data.ptr;
  out.in16 = data->byte1;
  return true;
}

bool GenD2(PointerList<EventTrigger>& triggers, EventData& out) {
  d.print(DBG_INFO, "GenD2");
  D_DATA* data = triggers.list[0]->data.ptr;
  out.in16 = data->byte2;
  return true;
}

bool PrintE(PointerList<EventTrigger>& triggers, EventData& out) {
  d.print(DBG_INFO, "PrintE");

  for (int trigIdx = 0; trigIdx < triggers.count; trigIdx++) {
    EventTrigger* trig = triggers.list[trigIdx];
    if (trig->data_ready && trig->event_id.equalsIgnoreCase(String("B"))) {
      d.print(DBG_INFO, "  B: %x", trig->data.in16);
    } else if (trig->data_ready && trig->event_id.equalsIgnoreCase(String("C"))) {
      d.print(DBG_INFO, "  C: %x", trig->data.in16);
    }
  }

  return false;
}

bool PrintF(PointerList<EventTrigger>& triggers, EventData& out) {
  d.print(DBG_INFO, "PrintF");

  for (int trigIdx = 0; trigIdx < triggers.count; trigIdx++) {
    EventTrigger* trig = triggers.list[trigIdx];
    if (trig->data_ready && trig->event_id.equalsIgnoreCase(String("A"))) {
      d.print(DBG_INFO, "  A: %x", trig->data.in16);
    } else if (trig->data_ready && trig->event_id.equalsIgnoreCase(String("B"))) {
      d.print(DBG_INFO, "  B: %x", trig->data.in16);
    } else if (trig->data_ready && trig->event_id.equalsIgnoreCase(String("D2"))) {
      d.print(DBG_INFO, "  D2: %x", trig->data.in16);
    }
  }

  return false;
}

// If this were declared before the EventHandlerFuncs, they could call it--but this way is less reentrant?
EventHub hub;

void LoopMessageAction() {
  static unsigned long LoopTimePassed = 0;
  LoopTimePassed = millis();
  d.print(DBG_INFO, "loop! %d seconds have passed", LoopTimePassed/1000);
}
DeltaClockEntry LoopMessage = {
  .action = LoopMessageAction,
  .interval = 3000,
  .repeating = true,
};
DeltaClock clock;

void setup() {
  Serial.begin(115200);
  d.print(DBG_INFO, "Begin Eventing Validation");

  { // Poke EventHandler
    EventData outData, inData;

    EventHandler h1(String("hi"), GenA, TRIGGER_ON_ANY, 0);
    h1.AddTrigger(String("bye"));
    inData.in16 = 0x01;
    outData.in16 = 0xFF;
    bool gend = h1.Deliver(String("bye"), inData, outData);
    d.print(DBG_INFO, "data generated %d %p", gend, outData.ptr);
  }

  d.print(DBG_INFO, "----------------------");

  hub.begin();
  hub.AddHandler(String("A"), GenA, TRIGGER_TEMPORAL, 1000);
  hub.AddHandler(String("C"), AtoC, TRIGGER_ON_ALL);
  hub.AddHandlerTrigger(String("C"), String("A"));
  hub.AddHandler(String("D"), GenD, TRIGGER_TEMPORAL, 3000);
  hub.AddHandler(String("D1"), GenD1, TRIGGER_ON_ALL);
  hub.AddHandlerTrigger(String("D1"), String("D"));
  hub.AddHandler(String("D2"), GenD2, TRIGGER_ON_ANY);
  hub.AddHandlerTrigger(String("D2"), String("D"));
  hub.AddHandler(String("E"), PrintE, TRIGGER_ON_ALL);
  hub.AddHandlerTrigger(String("E"), String("B"));
  hub.AddHandlerTrigger(String("E"), String("C"));
  hub.AddHandler(String("F"), PrintF, TRIGGER_ON_ANY);
  hub.AddHandlerTrigger(String("F"), String("A"));
  hub.AddHandlerTrigger(String("F"), String("B"));
  hub.AddHandlerTrigger(String("F"), String("D2"));

  hub.print();
  d.print(DBG_INFO, "----------------------");

  clock.insert(&LoopMessage);
  clock.begin();
}

void loop() {
  clock.update();
  hub.update();

  if (millis() % 1200 == 0) {
    d.print(DBG_INFO, "GenB");
    EventData bData;
    bData.in16 = 0xace5;
    hub.Deliver(String("B"), bData);
  }
}
