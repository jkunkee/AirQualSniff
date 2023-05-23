
#pragma once

//
// Copyright (C) 2022 Jon Kunkee jonathan.kunkee@gmail.com
// License: BSD 3-clause
//
// Event-driven programming library for Arduino
//
// (Probably trivially adaptable to any setup+loop C++ application)
//
// An Event is something that happens.
//
// Events are named in and referenced using EventNameEnum.
//
// Connections between Events are described in EventHandlers, which contain
// the set of Events the EventHandler depends on and how to wait for those
// Events. They also contain a pointer to an EventHandlerFunc function that
// executes when the trigger conditions are met.
//
// EventHandlerFunc functions can, in turn, produce data and emit Events.
//
// No handling is currently provided to enforce it, but the directed Event
// dependency graph should be acyclic.
//
// Data produced by an Event is copied into each listening EventHandler
// in the list of triggers. Multiple data types are accommodated using
// the EventData class; typically each Event name is associated with
// one data type, which is then implicitly coded into the EventHandlerFuncs.
// More complex data can be passed by having a global struct that is passed
// by pointer in EventData.ptr, but this moves the data flow outside
// the Eventing library's control.
//
// TODO:
// Pass by reference instead of value as many places as makes sense
// Use function pointers or enum as ID insead of String
// no-alloc/pre-alloc option
// debug dump of full state
// pass handler func the full handler object to make event searching easy
// shift type scopes inward to simplify interface
// somehow tidy up EventHandlerFunc interface
// make dbg.print levels meaningful (or hidden behind a macro)
//

#ifdef EVENTHUB_DEBUG
#ifdef PARTICLE_WIRING_ARDUINO_COMPATIBILTY
#define dbgprint(a, ...) Serial.printlnf(__VA_ARGS__)
#elif defined(ARDUINO)
// https://github.com/arduino-libraries/Arduino_DebugUtils/blob/master/src/Arduino_DebugUtils.h
#include <Arduino_DebugUtils.h>
static Arduino_DebugUtils eventhub_arduino_dbg;
#define dbgprint eventhub_arduino_dbg.print
#endif
#else
#define dbgprint(a, ...)
#endif

// https://arduino.stackexchange.com/questions/17639/the-difference-between-time-t-and-datetime#:~:text=A%20DateTime%20is%20a%20full%20class%20with%20lots,of%20the%20time%20stored%20in%20the%20DateTime%20object.
#ifndef time_t
//typedef unsigned long time_t;
#endif

#include "jet.h"

#ifdef EVENTHUB_TEMPORAL
#include "DeltaClock.h"
#endif

namespace Eventing {

typedef union _EventData {
  float fl;
  int16_t in16;
  uint16_t uin16;
  void* ptr;
} EventData;

typedef enum _EventTriggerType {
    TRIGGER_MANUAL,
    TRIGGER_ON_ANY,
    TRIGGER_ON_ALL,
#ifdef EVENTHUB_TEMPORAL
    TRIGGER_TEMPORAL,
#endif
} EventTriggerType;

class EventTrigger {
  // James: maybe allow for decimation here with 'wait until N fires'
public:
  EventTrigger(String id) : event_id(id), data({0}), data_ready(false) {}
  String event_id;
  EventData data;
  bool data_ready;
  void Deliver(EventData dataIn) {
    dbgprint(DBG_INFO, "    EventTrigger %s got %p", event_id.c_str(), data.ptr);
    data = dataIn;
    data_ready = true;
  }
  void Reset() {
#ifdef EVENTHUB_DEBUG
    data.in16 = 5;
#endif
    data_ready = false;
  }
#ifdef EVENTHUB_DEBUG
  void print() {
    dbgprint(DBG_INFO, "    EventTrigger %p listening for %s", this, event_id.c_str());
  }
#endif
};

typedef jet::PointerList<EventTrigger> EventTriggerList;

// The called function has no way to know how to generate an event, so
// rely on the parent EventHandler's ID string and filling out an output object
// to allow it to do so.
// Return true if data was generated.
typedef bool (*EventHandlerFunc)(EventTriggerList& triggers, EventData& out);

class EventHandler {
private:
public:
  EventHandler(String id, EventHandlerFunc func, EventTriggerType typeIn, time_t intervalIn = 0)
  : event_id(id)
  {
    action = func;
    type = typeIn;
    interval = intervalIn;
  }

  String event_id;
  EventHandlerFunc action;
  EventTriggerType type;
  EventTriggerList triggers;
  time_t interval;

  EventTrigger* FindTrigger(String id) {
    for (size_t triggerIdx = 0; triggerIdx < triggers.size(); triggerIdx++) {
      EventTrigger* trigger = triggers.get(triggerIdx);
      if (trigger->event_id.equalsIgnoreCase(id)) {
        return trigger;
      }
    }
    return nullptr;
  }
  // TODO: sweep to pass by reference instead of value
  bool AddTrigger(String id) {
    EventTrigger* trigger = FindTrigger(id);
    if (trigger == NULL) {
      return triggers.append(new EventTrigger(id));
    } else {
      return true;
    }
  }
  void ResetTriggers() {
    for (size_t triggerIdx = 0; triggerIdx < triggers.size(); triggerIdx++) {
      triggers.get(triggerIdx)->Reset();
    }
  }
  bool TriggerConditionMet() {
    switch (type) {
    default:
    case TRIGGER_MANUAL:
      return false;
#ifdef EVENTHUB_TEMPORAL
    case TRIGGER_TEMPORAL:
#endif
    case TRIGGER_ON_ANY:
      return true;
    case TRIGGER_ON_ALL: {
        bool AllTriggered = true;
        for (size_t triggerIdx = 0; triggerIdx < triggers.size() && AllTriggered; triggerIdx++) {
          AllTriggered = AllTriggered && triggers.get(triggerIdx)->data_ready;
        }
        return AllTriggered;
      }
    }
    return false;
  }
  // return true if this EventHandler fired and generated data
  bool Deliver(String id, EventData data, EventData& outData) {
    EventTrigger* trigger = FindTrigger(id);
    if (trigger == nullptr) {
      dbgprint(DBG_INFO, "  EventHandler %s did not find entry for trigger %s", event_id.c_str(), id.c_str());
      return false;
    }

    dbgprint(DBG_INFO, "  EventHandler %s got %s=%p", event_id.c_str(), id.c_str(), data.ptr);
    trigger->Deliver(data);

    if (TriggerConditionMet()) {
      dbgprint(DBG_INFO, "    TriggerConditionMet!");
      bool dataGenerated = action(triggers, outData);
      ResetTriggers();
      return dataGenerated;
    }

    return false;
  }
#ifdef EVENTHUB_DEBUG
  void print() {
    dbgprint(DBG_INFO, "  Handler %s type %d has %d triggers:", event_id.c_str(), type, triggers.count);
    for (int idx = 0; idx < triggers.count; idx++) {
      triggers.list[idx]->print();
    }
  }
#endif
};

class EventHub {
private:
#ifdef EVENTHUB_TEMPORAL
  DeltaClock clock;
#endif
  jet::PointerList<EventHandler> handlers;

  EventHandler* FindHandler(String id) {
    for (size_t handlerIdx = 0; handlerIdx < handlers.size(); handlerIdx++) {
      EventHandler* handler = handlers.get(handlerIdx);
      if (handler->event_id.equalsIgnoreCase(id)) {
        return handler;
      }
    }
    return nullptr;
  }

  class TemporalContext {
  public:
    TemporalContext() {}
    EventHandler* handler;
    EventHub* hub;
  };

public:
#ifdef EVENTHUB_TEMPORAL
  bool begin() {
    return clock.begin();
  }
  void update() {
    clock.update();
  }
  // If it's not static, the this pointer means it can't be a DeltaClockAction.
  // If it's static, it can't deliver the result.
  // Pack enough context into the void* to solve this problem!
  static void TemporalAction(void* contextIn) {
    dbgprint(DBG_INFO, "Firing Temporal Handler");
    TemporalContext* context = (TemporalContext*)contextIn;
    EventData data;
    bool produced = context->handler->action(context->handler->triggers, data);
    if (produced != false) {
      context->hub->Deliver(context->handler->event_id, data);
    }
  }
#endif
  bool AddHandler(String id, EventHandlerFunc func, EventTriggerType type, time_t interval = 0) {
    EventHandler* eventHandler = FindHandler(id);
    bool addStatus;
    if (eventHandler == nullptr) {
      eventHandler = new EventHandler(id, func, type, interval);
      addStatus = handlers.append(eventHandler);
#ifdef EVENTHUB_TEMPORAL
      // could just be at the end, but I don't remember how DeltaClock handles
      // double-add
      if (type == TRIGGER_TEMPORAL) {
        dbgprint(DBG_INFO, "Adding Temporal Handler");
        DeltaClockEntry* clockEntry = (DeltaClockEntry*)malloc(sizeof(DeltaClockEntry));
        clockEntry->action = (DeltaClockAction)&this->TemporalAction;
        clockEntry->interval = interval;
        clockEntry->repeating = true;
        TemporalContext* context = new TemporalContext();
        context->handler = eventHandler;
        context->hub = this;
        clockEntry->context = (void*)context;
        clock.insert(clockEntry);
      }
#endif
    } else {
      eventHandler->action = func;
      eventHandler->type = type;
      eventHandler->interval = interval;
      addStatus = true;
    }
    return addStatus;
  }
  bool AddHandlerTrigger(String handlerId, String triggerId) {
    EventHandler* eventHandler = FindHandler(handlerId);
    if (eventHandler != nullptr) {
      return eventHandler->AddTrigger(triggerId);
    } else {
      return false;
    }
  }
  bool Deliver(String id, EventData data) {
    dbgprint(DBG_INFO, "EventHub delivering %s %p at %lu", id.c_str(), data.ptr, millis());
    for (size_t handlerIdx = 0; handlerIdx < handlers.size(); handlerIdx++) {
      EventHandler* eventHandler = handlers.get(handlerIdx);
      EventData outData;
      bool eventFired = eventHandler->Deliver(id, data, outData);
      if (eventFired) {
        dbgprint(DBG_INFO, " Handler %s generated data: %p", eventHandler->event_id.c_str(), outData.ptr);
        Deliver(eventHandler->event_id, outData);
      }
    }
    dbgprint(DBG_INFO, "  EventHub delivery complete at %lu", millis());
    return true;
  }
#ifdef EVENTHUB_DEBUG
  void print() {
    dbgprint(DBG_INFO, "EventHub %p has %d handlers:", this, handlers.count);
    for (int idx = 0; idx < handlers.count; idx++) {
      handlers.list[idx]->print();
    }
  }
#endif
  void ToString(String& outString) {
    // Make a string snapshot of the hub state
    outString += "EventHub\n";
    outString += "Handlers:\n";
    for (int handlerIdx = 0; handlerIdx < (signed)handlers.size(); handlerIdx++) {
      EventHandler* handler = handlers.get(handlerIdx);
      outString += handlerIdx;
      outString += ":";
      outString += handler->event_id;
      outString += "\n";
      for (int triggerIdx = 0; triggerIdx < (signed)handler->triggers.size(); triggerIdx++) {
        outString += handlerIdx;
        outString += ":";
        outString += triggerIdx;
        outString += ">";
        EventTrigger* trigger = handler->triggers.get(triggerIdx);
        outString += trigger->event_id;
        outString += ",rdy=";
        outString += trigger->data_ready;
        outString += "\n";
      }
    }
    outString += "DeltaClock:\n";
    clock.ToString(outString);
  }
};

} // namespace Eventing
