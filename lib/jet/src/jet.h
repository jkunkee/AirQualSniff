
#pragma once

//
// Copyright (C) 2023 Jon Kunkee jonathan.kunkee@gmail.com
// License: BSD 3-clause
//
// jet: Jon's Embedded Library
// A collection of data structures and algorithms I find generally helpful when
// programming for embedded devices.
//
// Why not use the standard library? What about someone else's library?
//
// It comes down to "I'm picky", "I'm not getting paid for this", and "this is
// fun". I wanted a library that was:
// * Barely engineered, not overengineered -- so easy to understand
// * Not using virtual functions
// * Not using templates except How I Like To for Reasons
// * Available and compatible wherever I want it
// * Easy to use with static allocations
// * Easy to add to a project
// * Fit to my tasks
// * Testable
//
// Table of Contents
// * logging and testing ifdefs
// * namespace jet
//   * PointerList<T>
//     - A simple list-of-pointers class backed by an array.
//     - Storage can be allocated externally or managed internally using the heap.
//     - Internal array management only grows the storage by a fixed amount per overflow.
//     - Stored pointers are never freed or allocated -- this is left to the caller.
//   * PointerListTest()
//   * namespace evt
//     * DeltaClock
//       - A "delta clock", a straightforward and efficient event-scheduling data structure.
//       - This implementation is not thread-safe, but the concept is intended for performant use in interrupt handlers.
//       - Storage is entirely externally managed.
//       - Time is managed externally as well.
//     * DeltaClockTest
//     * HubTest
//

#ifdef JET_TEST

#ifdef PARTICLE_WIRING_ARDUINO_COMPATIBILTY
#define jet_dbgprint(...) Serial.printlnf(__VA_ARGS__)
#elif defined(ARDUINO)
// https://github.com/arduino-libraries/Arduino_DebugUtils/blob/master/src/Arduino_DebugUtils.h
#include <Arduino_DebugUtils.h>
Arduino_DebugUtils eventhub_arduino_dbg;
#define jet_dbgprint(...) eventhub_arduino_dbg.print(0, __VA_ARGS__)
#endif

#ifdef JET_TEST_TRACING
#define jet_traceprint jet_dbgprint
#else
#define jet_traceprint(...)
#endif

#define jet_assert_var bool success = true;
#define jet_assert(expr) \
  if (success && !(expr)) { \
    success = false; \
    jet_dbgprint("Assertion failed: '%s' at line %d", #expr, __LINE__); \
  }

#else // JET_TEST
#define jet_dbgprint(...)
#define jet_traceprint(...)
#endif // JET_TEST

// https://arduino.stackexchange.com/questions/17639/the-difference-between-time-t-and-datetime#:~:text=A%20DateTime%20is%20a%20full%20class%20with%20lots,of%20the%20time%20stored%20in%20the%20DateTime%20object.
#if !defined(PARTICLE_WIRING_ARDUINO_COMPATIBILTY) && defined(ARDUINO) && !defined(time_t)
typedef unsigned long time_t;
#endif

namespace jet {

// Pointer List
// does not hold null
// memory-friendly linear buffer growth
// static buffer support

template <class T> class PointerList {
private:
  T** m_list;
  size_t m_count;
  size_t m_capacity;
  bool m_list_is_internally_managed;
public:
  PointerList(T** buf, size_t buf_elems) :
    m_list(buf),
    m_count(0),
    m_capacity(buf_elems),
    m_list_is_internally_managed(false)
  {}
  PointerList(size_t initial_capacity) :
    m_count(0),
    m_list_is_internally_managed(true)
  {
    m_list = (T**)malloc(sizeof(T*)*initial_capacity);
    m_capacity = initial_capacity;
    if (m_list == nullptr) {
      m_capacity = 0;
    }
  }
  PointerList() : PointerList(5) {}
  ~PointerList()
  {
    if (m_list_is_internally_managed) {
      free(m_list);
    }
  }
  bool insert(int idx, T* item)
  {
    jet_traceprint("insert idx:%d item:%p", idx, item);
    if (item == nullptr) {
      jet_dbgprint("insert null check failed");
      return false;
    }
    // Insertion can land at the end (-1) or at any existing index
    if (idx == -1) {
      idx = m_count;
    }
    if (idx < 0 || (signed)m_count < idx) {
      jet_dbgprint("insert range check failed");
      return false;
    }
    // Is there room?
    if (m_count + 1 > m_capacity) {
      if (m_list_is_internally_managed) {
      jet_dbgprint("insert increase needed");
        // Allocate a bigger buffer.
        set_capacity(m_capacity + 2);
      } else {
        jet_dbgprint("insert size check failed");
        // We don't own the buffer, so fail and let the caller deal with it.
        return false;
      }
    }

    // Move the remainder down
    for (int dest_idx = m_count; dest_idx > idx; dest_idx--) {
      int src_idx = dest_idx - 1;
      m_list[dest_idx] = m_list[src_idx];
    }
    // Insert new item
    m_list[idx] = item;
    m_count += 1;
    return true;
  }
  bool append(T* item) {
    jet_traceprint("append %p", item);
    return insert(-1, item);
  }
  bool replace(int idx, T* item) {
    if (item == nullptr) {
      return false;
    }
    if (idx < 0 || m_count <= idx) {
      return false;
    }
    m_list[idx] = item;
  }
  T* get(int idx) {
    if (idx < 0 || (signed)m_count <= idx) {
      return nullptr;
    }
    return m_list[idx];
  }
  bool remove(int idx) {
    if (idx < -1 || (signed)m_count <= idx) {
      jet_dbgprint("remove failed range check %d", idx);
      return false;
    }
    jet_traceprint("remove %d", idx);
    if (idx == -1) {
      idx = m_count-1;
    }
    for (unsigned int src_idx = idx + 1; src_idx < m_count; src_idx++) {
      int dst_idx = src_idx - 1;
      jet_traceprint("remove collapse %d<-%d", dst_idx, src_idx);
      m_list[dst_idx] = m_list[src_idx];
    }
    m_count -= 1;
    return true;
  }
  int find_first(T* item) {
    for (unsigned int idx = 0; idx < m_count; idx++) {
      if (m_list[idx] == item) {
        return idx;
      }
    }
    return -1;
  }
  bool remove_first(T* item) {
    int idx = find_first(item);
    if (idx == -1) {
      return false;
    }
    return remove(find_first(item));
  }
  bool swap(int idx1, int idx2) {
    if (idx1 < 0 || m_count <= idx1 || idx2 < 0 || m_count <= idx2) {
      return false;
    }
    T* temp = m_list[idx1];
    m_list[idx1] = m_list[idx2];
    m_list[idx2] = temp;
    return true;
  }
  void clear() { m_count = 0; }

  bool is_full() { return m_count == m_capacity; }
  bool is_empty() { return m_count == 0; }
  size_t size() { return m_count; }
  size_t capacity() { return m_capacity; }
  bool set_capacity(size_t new_capacity) {
    if (!m_list_is_internally_managed) {
      return false;
    }
    if (m_count > new_capacity) {
      return false;
    }
    T** new_buf = (T**)malloc(sizeof(T*) * new_capacity);
    memcpy(new_buf, m_list, m_count * sizeof(T*));
    free(m_list);
    m_list = new_buf;
    m_capacity = new_capacity;
    return true;
  }
#ifdef JET_TEST
  friend bool PointerListTest();
#endif
};

#ifdef JET_TEST

bool PointerListTest() {
  jet_assert_var;
  PointerList<int>* list;

  int mem[4];
  int* ptrs[4];

  list = new PointerList<int>(ptrs, 4);
  jet_assert(list->m_capacity == 4);
  jet_assert(list->m_list == ptrs);
  jet_assert(list->m_list_is_internally_managed == false);
  jet_assert(list->m_count == 0);
  jet_assert(list->append(&mem[0]));
  jet_assert(list->m_count == 1);
  jet_assert(list->append(&mem[1]));
  jet_assert(list->m_count == 2);
  jet_assert(list->append(&mem[2]));
  jet_assert(list->m_count == 3);
  jet_assert(list->append(&mem[3]));
  jet_assert(list->m_count == 4);
  jet_assert(!list->append(&mem[0]));
  jet_assert(list->m_count == 4);
  delete(list);

  list = new PointerList<int>(4);
  jet_assert(list->m_capacity == 4);
  jet_assert(list->m_list != nullptr);
  jet_assert(list->m_list_is_internally_managed == true);
  jet_assert(list->is_empty());
  jet_assert(list->size() == 0);
  jet_assert(list->append(&mem[0]));
  jet_assert(list->size() == 1);
  jet_assert(list->m_list[0] == &mem[0]);
  jet_assert(list->append(&mem[1]));
  jet_assert(list->size() == 2);
  jet_assert(list->m_list[1] == &mem[1]);
  jet_assert(list->append(&mem[2]));
  jet_assert(list->size() == 3);
  jet_assert(list->append(&mem[3]));
  jet_assert(list->size() == 4);
  jet_assert(list->capacity() == 4);
  jet_assert(list->is_full());
  jet_assert(list->append(&mem[0]));
  jet_assert(list->size() == 5);
  jet_assert(list->capacity() == 4 + 2);
  list->clear();
  jet_assert(list->size() == 0);

  jet_assert(list->capacity() == 4 + 2);
  list->append(&mem[0]);
  list->append(&mem[1]);
  jet_assert(list->m_list[0] == &mem[0]);
  jet_assert(list->m_list[1] == &mem[1]);
  jet_assert(list->swap(0, 1));
  jet_assert(list->m_list[0] == &mem[1]);
  jet_assert(list->m_list[1] == &mem[0]);
  jet_assert(list->remove(0));
  jet_assert(list->size() == 1);
  jet_assert(list->m_list[0] == &mem[0]);
  jet_assert(list->remove(0));
  jet_assert(list->size() == 0);

  jet_assert(list->insert(0, &mem[0]));
  jet_assert(list->m_list[0] == &mem[0]);
  jet_assert(list->insert(0, &mem[1]));
  jet_assert(list->m_list[0] == &mem[1]);
  jet_assert(list->m_list[1] == &mem[0]);
  jet_assert(list->insert(1, &mem[2]));
  jet_assert(list->m_list[0] == &mem[1]);
  jet_assert(list->m_list[1] == &mem[2]);
  jet_assert(list->m_list[2] == &mem[0]);
  jet_assert(list->get(0) == &mem[1]);
  jet_assert(list->get(1) == &mem[2]);
  jet_assert(list->get(2) == &mem[0]);
  jet_assert(list->get(3) == nullptr);
  list->clear();

  list->insert(-1, &mem[0]);
  list->insert(-1, &mem[1]);
  list->insert(-1, &mem[2]);
  list->insert(-1, &mem[3]);
  jet_assert(list->find_first(&mem[2]) == 2);
  jet_assert(list->remove_first(&mem[2]));
  jet_assert(list->find_first(&mem[2]) == -1);
  jet_assert(list->find_first(&mem[3]) == 2);
  jet_assert(!list->remove_first(&mem[2]));
  jet_assert(list->find_first(&mem[3]) == 2);
  jet_assert(list->size() == 3);
  delete(list);

  if (success) {
    jet_dbgprint("Success!");
  } else {
    jet_dbgprint("Failure...");
  }

  return success;
}

#endif // JET_TEST


namespace evt {

class DeltaClock {
public:
  // child types
  typedef void (*Action)(void*);
  class Entry {
  public:
    // Execution
    Action action;
    // A void* context structure allows for reuse of a single DeltaClockAction
    // for multiple DeltaClockEntries--say, when an EventHub wraps their creation.
    void* context;
    // Bookkeeping
    time_t interval;
    boolean repeating;
    time_t remaining;
    Entry* next;
  };
private:
  Entry* m_head;
  time_t m_last_update;
  // constant for detecting most rollover, overflow, and underflow conditions
  const time_t m_max_interval = (((time_t)1) << (sizeof(time_t) * 8 - 1)) - 1;
public:
  DeltaClock() : m_head(nullptr), m_last_update(0) {}
  ~DeltaClock() { clear(); }
  // typically called with millis(), but works with any unsigned monotonic time value
  void update(time_t now) {
    // Constrain delta to be nonzero
    if (now == m_last_update) {
      jet_dbgprint("no time has passed (or exactly one max-time_t time interval has passed)");
      return;
    }
    time_t delta = now - m_last_update;
    // monotonic time counter rollover
    if (now < m_last_update) {
      jet_traceprint("DeltaClock monotonic timer wraparound");
      // type math validated in test suite
      delta = ((unsigned)-(signed)(m_last_update)) + now;
    }
    jet_traceprint("update from %lu", m_last_update);
    jet_traceprint("       to %lu", now);
    jet_traceprint("       delta %lu", delta);
    // Update entries
    while (m_head != NULL) {
      jet_traceprint("  delta:%lu ---------------", delta);
      jet_traceprint("  processing %p", m_head);
      jet_traceprint("  remaining:%lu", m_head->remaining);
      if (m_head->remaining > delta) {
        jet_traceprint("  charge and break");
        m_head->remaining -= delta;
        break;
      } else {
        jet_traceprint("  retire");
        // The entry has expired.
        Entry* entry = m_head;
        // Charge it against the delta.
        delta -= entry->remaining;
        entry->remaining = 0;
        // Dequeue it.
        m_head = entry->next;
        entry->next = nullptr;
        // Execute it.
        entry->action(entry->context);
        // Requeue if repeating
        if (entry->repeating) {
          jet_traceprint("  reschedule");
          schedule(entry);
        }
      }
    }
    m_last_update = now;
  }
  // If two events end up with the same expiration time, the first one inserted will run first.
  bool schedule(Entry* new_entry) {
    // Input validation
    if (new_entry == nullptr ||
        new_entry->interval == 0 ||
        new_entry->interval > m_max_interval ||
        new_entry->action == nullptr ||
        new_entry->next != nullptr) {
      jet_dbgprint("schedule failed with invalid arg");
      return false;
    }
    // Initialize
    new_entry->remaining = new_entry->interval;
    // Insert
    // Empty list
    if (m_head == nullptr) {
      jet_traceprint("schedule empty list case");
      m_head = new_entry;
      return true;
    }
    // Find insertion point
    Entry* prev = nullptr;
    Entry* next = m_head;
    // find insertion point; start at head
    while (true) {
      // if this is the insertion point, insert
      if (prev == nullptr && new_entry->remaining < next->remaining) {
        // beginning
        jet_traceprint("schedule at beginning");
        new_entry->next = next;
        m_head = new_entry;
        next->remaining -= new_entry->remaining;
        break;
      } else if (next == NULL) {
        // end
        jet_traceprint("schedule at end");
        prev->next = new_entry;
        break;
      } else if (new_entry->remaining < next->remaining) {
        // middle
        jet_traceprint("schedule in middle");
        prev->next = new_entry;
        new_entry->next = next;
        next->remaining -= new_entry->remaining;
        break;
      }
      // move to next entry
      new_entry->remaining -= next->remaining;
      prev = next;
      next = next->next;
    }
    return true;
  }
  //bool unschedule(DeltaClockEntry* entry);
  //time_t get_remaining_time(DeltaClockEntry* entry);
  void clear() {
    for (Entry* current = m_head; current != nullptr; current = current->next) {
      current->next = nullptr;
    }
    m_head = nullptr;
  }
  void debug_string(String& out) {
    out += "DeltaClock "+String((uintptr_t)this, HEX)+"\n";
    for (Entry* entry = m_head; entry != nullptr; entry = entry->next) {
      out += "  ";
      out += String((uintptr_t)entry, HEX);
      out += " rem: ";
      out += String((uint32_t)entry->remaining);
      out += " next: ";
      out += String((uintptr_t)entry->next, HEX);
      out += "\n";
    }
    out += "  End DeltaClock";
  }
#ifdef JET_TEST
  friend bool DeltaClockTest();
#endif // JET_TEST
};

#ifdef JET_TEST

#define COUNTER_ENTRY(id, ivl, rep) \
static time_t Counter##id; \
static void Action##id(void*); \
static DeltaClock::Entry Entry##id = { \
  .action = &Action##id, \
  .context = nullptr, \
  .interval = ivl, \
  .repeating = rep, \
}; \
static void Action##id(void*) { \
  Counter##id++; \
}
//  jet_dbgprint("  "#id":%d", Counter##id);
//  if (Counter##id > 70) { Entry##id.repeating = false; }

COUNTER_ENTRY(A, 1000, false)
COUNTER_ENTRY(B, 2000, false)
COUNTER_ENTRY(C, 1000, false)

COUNTER_ENTRY(1, 1000, true);
COUNTER_ENTRY(2, 10000, true);
COUNTER_ENTRY(3, 1000, true);
COUNTER_ENTRY(4, 1000, true);
COUNTER_ENTRY(5, 1000, true);
COUNTER_ENTRY(6, 12*60*60*1000, true);
COUNTER_ENTRY(7, 1200, true);

bool DeltaClockTest() {
  jet_assert_var;
  DeltaClock* clock;

  if (success) {
    jet_dbgprint("time type properties");
    time_t big_time = -1000;
    time_t small_time = 1000;
    jet_assert(sizeof(time_t) >= 4);
    jet_assert(big_time > small_time);
    jet_assert(((unsigned)-(signed)(big_time)) + small_time == 2000);
    // My old method does not work the way it was
    //jet_assert((time_t)(-1) - big_time + small_time == 2000);
  }

  clock = new DeltaClock();
  if (success) {
    jet_dbgprint("constructor");
    jet_assert(clock->m_head == nullptr);
    jet_assert(clock->m_last_update == 0);
  }
  if (success) {
    jet_dbgprint("empty list update");
    clock->update(5000);
    jet_assert(clock->m_last_update == 5000);
  }
  if (success) {
    jet_dbgprint("scheduling one event");
    jet_assert(clock->schedule(&EntryA));
    jet_assert(clock->m_head == &EntryA);
    jet_assert(clock->m_head->remaining == EntryA.interval);
    jet_assert(clock->m_head->next == nullptr);
  }
  if (success) {
    jet_dbgprint("schedule a second, following event");
    jet_assert(clock->schedule(&EntryB));
    jet_assert(clock->m_head == &EntryA);
    jet_assert(clock->m_head->remaining == EntryA.interval);
    jet_assert(clock->m_head->next == &EntryB);
    jet_assert(clock->m_head->next->remaining == EntryB.interval - EntryA.interval);
  }
  if (success) {
    jet_dbgprint("clear the list, including next pointers");
    clock->clear();
    jet_assert(clock->m_head == nullptr);
    jet_assert(EntryA.next == nullptr);
    jet_assert(EntryB.next == nullptr);
  }
  if (success) {
    jet_dbgprint("schedule a second, earlier event");
    jet_assert(clock->schedule(&EntryB));
    jet_assert(clock->m_head == &EntryB);
    jet_assert(clock->m_head->remaining == EntryB.interval);
    jet_assert(clock->schedule(&EntryA));
    jet_assert(clock->m_head == &EntryA);
    jet_assert(clock->m_head->remaining == EntryA.interval);
    jet_assert(clock->m_head->next == &EntryB);
    jet_assert(clock->m_head->next->remaining == EntryB.interval - EntryA.interval);
  }
  if (success) { jet_dbgprint("destructor"); }
  delete(clock);
  jet_assert(EntryA.next == nullptr);
  jet_assert(EntryB.next == nullptr);

  clock = new DeltaClock();
  if (success) {
    jet_dbgprint("fire both");
    jet_assert(clock->schedule(&EntryA));
    jet_assert(clock->schedule(&EntryB));
    CounterA = 0;
    CounterB = 0;
    clock->update(100);
    jet_assert(clock->m_head->remaining == clock->m_head->interval - 100);
    clock->update(200);
    jet_assert(clock->m_head->remaining == clock->m_head->interval - 200);
    clock->update(1000);
    jet_assert(CounterA == 1);
    jet_assert(CounterB == 0);
    jet_assert(EntryA.next == nullptr);
    clock->update(2000);
    jet_assert(CounterA == 1);
    jet_assert(CounterB == 1);
    jet_assert(EntryA.next == nullptr);
    jet_assert(EntryB.next == nullptr);
  }
  if (success) {
    jet_dbgprint("repeater requeue");
    EntryA.repeating = true;
    EntryB.repeating = true;
    clock->m_last_update = 0;
    jet_assert(clock->schedule(&EntryA));
    jet_assert(clock->schedule(&EntryB));
    CounterA = 0;
    CounterB = 0;
    clock->update(1000);
    jet_assert(CounterA == 1);
    jet_assert(CounterB == 0);
    jet_assert(clock->m_head == &EntryB);
    jet_assert(EntryA.next == nullptr);
    jet_assert(EntryB.next == &EntryA);
    clock->update(2000);
    jet_assert(CounterA == 2);
    jet_assert(CounterB == 1);
    jet_assert(clock->m_head == &EntryA);
    jet_assert(EntryA.next == &EntryB);
    jet_assert(EntryB.next == nullptr);
  }
  if (success) {
    jet_dbgprint("repeater repeats");
    clock->update(15000);
    jet_assert(CounterA == 15);
    jet_assert(CounterB == 7);
    jet_assert(clock->m_head == &EntryB);
    jet_assert(EntryA.next == nullptr);
    jet_assert(EntryB.next == &EntryA);
  }
  if (success) {
    jet_dbgprint("simultaneous events");
    clock->clear();
    clock->m_last_update = 0;
    CounterA = 0;
    EntryA.repeating = true;
    CounterC = 0;
    EntryC.repeating = true;
    jet_assert(clock->schedule(&EntryA));
    jet_assert(clock->schedule(&EntryC));
    jet_assert(clock->m_head == &EntryA);
    jet_assert(EntryA.next == &EntryC);
    jet_assert(EntryA.remaining == 1000);
    jet_assert(EntryC.remaining == 0);
    clock->update(1000);
    jet_assert(clock->m_head == &EntryA);
    jet_assert(EntryA.next == &EntryC);
    jet_assert(EntryA.remaining == 1000);
    jet_assert(EntryC.remaining == 0);
    jet_assert(CounterA == 1);
    jet_assert(CounterC == 1);
  }
  if (success) {
    jet_dbgprint("partial repeater costing");
    clock->clear();
    clock->m_last_update = 0;
    CounterA = 0;
    EntryA.interval = 1000;
    EntryA.repeating = true;
    CounterB = 0;
    EntryB.interval = 1000;
    EntryB.repeating = true;
    jet_assert(clock->schedule(&EntryA));
    jet_assert(clock->schedule(&EntryB));
    clock->update(200);
    jet_assert(EntryA.remaining == 800);
    jet_assert(EntryB.remaining == 0);
    clock->update(1200);
    jet_assert(CounterA == 1);
    jet_assert(CounterB == 1);
    jet_assert(EntryA.remaining == 800);
    jet_assert(EntryB.remaining == 0);
  }
  if (success) {
    jet_dbgprint("timer overflow");
    clock->clear();
    clock->m_last_update = 0;
    CounterA = 0;
    EntryA.interval = 1000;
    EntryA.repeating = true;
    clock->update(UINT32_MAX-500);
    jet_assert(clock->schedule(&EntryA));
    clock->update(500-1);
    jet_assert(CounterA == 1);
    jet_assert(EntryA.remaining = 1000);
  }
  if (success) {
    jet_dbgprint("periodically simultaneous events");
    clock->clear();
    clock->m_last_update = 0;
    CounterA = 0;
    EntryA.interval = 1000;
    EntryA.repeating = true;
    CounterB = 0;
    EntryB.interval = 1000;
    EntryB.repeating = true;
    CounterC = 0;
    EntryC.interval = 1200;
    EntryC.repeating = true;
    jet_assert(clock->schedule(&EntryA));
    jet_assert(clock->schedule(&EntryB));
    jet_assert(clock->schedule(&EntryC));
    clock->update(60UL*1000UL);
    jet_assert(CounterA == 60);
    jet_assert(CounterB == 60);
    jet_assert(CounterC == 50);
  }
  if (success) {
    jet_dbgprint("AirQualSniff Scenario");
    clock->clear();
    clock->m_last_update = 0;
    clock->schedule(&Entry1);
    clock->schedule(&Entry2);
    clock->schedule(&Entry3);
    clock->schedule(&Entry4);
    clock->schedule(&Entry5);
    clock->schedule(&Entry6);
    clock->schedule(&Entry7);
    Counter1 = 0;
    Counter2 = 0;
    Counter3 = 0;
    Counter4 = 0;
    Counter5 = 0;
    Counter6 = 0;
    Counter7 = 0;
    time_t test_duration = 36UL*60UL*60UL*1000UL;
    clock->update(test_duration);
    jet_assert(Counter1 == (test_duration / Entry1.interval));
    jet_assert(Counter2 == (test_duration / Entry2.interval));
    jet_assert(Counter3 == (test_duration / Entry3.interval));
    jet_assert(Counter4 == (test_duration / Entry4.interval));
    jet_assert(Counter5 == (test_duration / Entry5.interval));
    jet_assert(Counter6 == (test_duration / Entry6.interval));
    jet_assert(Counter7 == (test_duration / Entry7.interval));
    if (!success) {
      jet_dbgprint("Counter1: %lu / %lu", Counter1, test_duration / Entry1.interval);
      jet_dbgprint("Counter2: %lu / %lu", Counter2, test_duration / Entry2.interval);
      jet_dbgprint("Counter3: %lu / %lu", Counter3, test_duration / Entry3.interval);
      jet_dbgprint("Counter4: %lu / %lu", Counter4, test_duration / Entry4.interval);
      jet_dbgprint("Counter5: %lu / %lu", Counter5, test_duration / Entry5.interval);
      jet_dbgprint("Counter6: %lu / %lu", Counter6, test_duration / Entry6.interval);
      jet_dbgprint("Counter7: %lu / %lu", Counter7, test_duration / Entry7.interval);
    }
  }
  if (!success) {
    String str;
    clock->debug_string(str);
    jet_dbgprint("%s", str.c_str());
  }
  delete(clock);

  if (!success) {
    jet_dbgprint("EntryA: %p", &EntryA);
    jet_dbgprint("EntryB: %p", &EntryB);
    jet_dbgprint("EntryC: %p", &EntryC);
  }
  return success;
}

#endif // JET_TEST


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

class Trigger;
class Event;
class Hub;

typedef jet::PointerList<Trigger> TriggerList;
typedef jet::PointerList<Event> EventList;

// Data transfer format
typedef union _Datum {
  float fl;
  int16_t in16;
  uint16_t uin16;
  void* ptr;
} Datum;

// Function to call when an event fires.
//
// The called function has no way to know how to generate an event, so
// rely on the parent EventHandler's ID string and filling out an output object
// to allow it to do so.
// Return true if data was generated.
typedef bool (*HandlerFunc)(TriggerList& triggers, Datum& out);

#ifndef EventIndex
#define EventIndex String
#endif

typedef enum _TriggerType {
    TRIGGER_MANUAL,
    TRIGGER_ON_ANY,
    TRIGGER_ON_ALL,
#ifdef JET_EVT_HUB_TEMPORAL
    TRIGGER_TEMPORAL,
#endif
} TriggerType;

class Trigger {
  // James: maybe allow for decimation here with 'wait until N fires'
public:
  Trigger(EventIndex id) : event_id(id), event_name(id), data({0}), data_ready(false) {}
  EventIndex event_id;
  String event_name;
  Datum data;
  bool data_ready;
  void deliver(Datum data_in) {
    jet_traceprint("    EventTrigger %s got %p", event_name.c_str(), data.ptr);
    data = data_in;
    data_ready = true;
  }
  void reset() {
#ifdef JET_TEST
    data.uin16 = 0xdead;
#endif
    data_ready = false;
  }
#ifdef JET_TEST
  void debug_string(String& out);
  friend bool HubTest();
#endif
};

class Event {
private:
  String event_name;
  HandlerFunc action;
  TriggerType type;
  TriggerList triggers;
  time_t interval;
public:
  EventIndex event_id;
  Event(EventIndex id, HandlerFunc func, TriggerType type_in, time_t interval_in = 0) :
    event_id(id), event_name(id), action(func), type(type_in), interval(interval_in) {}
  ~Event() {
    for (int trig_idx = 0; trig_idx < triggers.size(); trig_idx++) {
      delete(triggers.get(trig_idx));
    }
  }
  Trigger* find_trigger(EventIndex id) {
    for (int trig_idx = 0; trig_idx < triggers.size(); trig_idx++) {
      Trigger* trigger = triggers.get(trig_idx);
      if (trigger->event_id == id) {
        return trigger;
      }
    }
    return nullptr;
  }
  bool add_trigger(EventIndex id) {
    Trigger* trigger = find_trigger(id);
    if (trigger == nullptr) {
      trigger = new Trigger(id);
      triggers.append(trigger);
      return true;
    } else {
      return false;
    }
  }
  void reset_triggers() {
    for (int trig_idx = 0; trig_idx < triggers.size(); trig_idx++) {
      triggers.get(trig_idx)->reset();
    }
  }
  bool trigger_condition_met() {
    switch (type) {
    default:
    case TRIGGER_MANUAL:
#ifdef JET_EVT_HUB_TEMPORAL
    case TRIGGER_TEMPORAL:
#endif
      return false;
    case TRIGGER_ON_ANY:
      for (int trig_idx = 0; trig_idx < triggers.size(); trig_idx++) {
        if (triggers.get(trig_idx)->data_ready) {
          return true;
        }
      }
      return false;
    case TRIGGER_ON_ALL:
      for (int trig_idx = 0; trig_idx < triggers.size(); trig_idx++) {
        if (!triggers.get(trig_idx)->data_ready) {
          return false;
        }
      }
      return true;
    }
    return false;
  }
  bool take_action(Datum& result) {
    bool data_generated = action(triggers, result);
    reset_triggers();
    return data_generated;
  }
  bool deliver_trigger(EventIndex id, Datum input, Datum& result) {
    Trigger* trigger = find_trigger(id);
    if (trigger != nullptr && trigger->event_id == id) {
      trigger->data = input;
      trigger->data_ready = true;
      if (trigger_condition_met()) {
        return take_action(result);
      }
    }
    return false;
  }
#ifdef JET_TEST
  void debug_string(String& out);
  friend bool HubTest();
#endif
};

class Hub {
private:
  EventList m_event_list;
  Event* find_event(EventIndex id) {
    for (int idx = 0; idx < m_event_list.size(); idx++) {
      Event* event = m_event_list.get(idx);
      if (event->event_id == id) {
        return event;
      }
    }
    return nullptr;
  }
#ifdef JET_EVT_HUB_TEMPORAL
  DeltaClock clock;
  PointerList<void> m_clock_entries;
  PointerList<void> m_clock_handler_contexts;
  // Generic DeltaClock Event actions can take a void* context.
  // Hub DeltaClock events use a wrapper action that lets the actions fire events.
  typedef struct _TemporalContext {
    Event* event;
    Hub* hub;
  } TemporalContext;
  static void TemporalAction(void* context_in) {
    TemporalContext* context = (TemporalContext*)context_in;
    Datum datum;
    bool produced = context->event->take_action(datum);
    if (produced != false) {
      //context->hub->deliver(context->event->event_id, datum);
    }
  }
#endif
public:
  Hub() {}
  ~Hub() {
    for (int evt_idx = 0; evt_idx < m_event_list.size(); evt_idx++) {
      delete(m_event_list.get(evt_idx));
    }
#ifdef JET_EVT_HUB_TEMPORAL
    for (int idx = 0; idx < m_clock_entries.size(); idx++) {
      free(m_clock_entries.get(idx));
    }
    for (int idx = 0; idx < m_clock_handler_contexts.size(); idx++) {
      free(m_clock_handler_contexts.get(idx));
    }
#endif
  }
#ifdef JET_EVT_HUB_TEMPORAL
  void update(time_t new_now) {
    clock.update(new_now);
  }
#endif
  bool add_event(EventIndex id, HandlerFunc func, TriggerType type, time_t interval = 0) {
    Event* event = find_event(id);
    if (event != nullptr) {
      return false; // TODO considering overwrite
    }
    event = new Event(id, func, type, interval);
    m_event_list.append(event);
#ifdef JET_EVT_HUB_TEMPORAL
    if (type == TRIGGER_TEMPORAL) {
      DeltaClock::Entry* clock_entry = new DeltaClock::Entry();
      m_clock_entries.append(clock_entry);
      clock_entry->action = &TemporalAction;
      TemporalContext* context = new TemporalContext();
      m_clock_handler_contexts.append(context);
      context->event = event;
      context->hub = this;
      clock_entry->context = context;
      clock_entry->interval = interval;
      clock_entry->repeating = true;
      return clock.schedule(clock_entry);
    }
#endif
    return true;
  }
  bool add_event_trigger(EventIndex event_id, EventIndex trigger_event_id) {
    Event* event = find_event(event_id);
    if (event != nullptr) {
      return event->add_trigger(trigger_event_id);
    }
    return false;
  }
  bool deliver(EventIndex event_id, Datum datum) {
    for (int event_idx = 0; event_idx < m_event_list.size(); event_idx++) {
      Datum product = { 0 };
      bool produced;
      Event* event = m_event_list.get(event_idx);
      produced = event->deliver_trigger(event_id, datum, product);
      if (produced) {
        deliver(event->event_id, product);
      }
    }
    return true;
  }
  bool is_dag();
#ifdef JET_TEST
  void debug_string(String& out);
  friend bool HubTest();
#endif
};

#ifdef JET_TEST

int TestHandlerCounter = 0;
bool TestHandler(TriggerList& triggers, Datum& out) {
  TestHandlerCounter++;
  return false;
}
bool TestHandlerProducer(TriggerList& triggers, Datum& out) {
  TestHandlerCounter++;
  out.in16 = TestHandlerCounter;
  return true;
}

bool HubTest() {
  jet_assert_var;

  if (success) {
    jet_dbgprint("hub trigger: constructor/destructor");
    Trigger* tr = new Trigger("hello");
    jet_assert(tr != nullptr);
    jet_assert(tr->event_id == "hello");
    jet_assert(tr->event_name.equals("hello"));
    jet_assert(tr->data.in16 == 0);
    jet_assert(tr->data_ready == false);
    delete(tr);
  }

  if (success) {
    jet_dbgprint("hub trigger: deliver/reset");
    Trigger* tr = new Trigger("TestBareTrigger");
    Datum d = { .in16 = 5 };
    if (success) {
      tr->deliver(d);
      jet_assert(tr->data.in16 == 5);
      jet_assert(tr->data_ready == true);
      tr->reset();
      jet_assert(tr->data.uin16 == 0xdead);
      jet_assert(tr->data_ready == false);
    }
    delete(tr);
  }

  if (success) {
    jet_dbgprint("hub event: constructor/destructor");
    Event* ev = new Event("Halooo", &TestHandler, TRIGGER_ON_ALL, 1000UL);
    jet_assert(ev->action == &TestHandler);
    jet_assert(ev->event_id == "Halooo");
    jet_assert(ev->event_name.equals("Halooo"));
    jet_assert(ev->interval == 1000UL);
    jet_assert(ev->type == TRIGGER_ON_ALL);
    jet_assert(ev->triggers.size() == 0);
    delete(ev);
  }

  if (success) {
    jet_dbgprint("hub event: add/find trigger");
    Event* ev = new Event("Halooo", &TestHandler, TRIGGER_ON_ANY);
    jet_assert(ev->add_trigger("Partisan"));
    jet_assert(ev->triggers.size() == 1);
    jet_assert(ev->triggers.get(0) != nullptr);
    jet_assert(ev->triggers.get(0)->event_name.equals("Partisan"));
    Trigger* tr = ev->find_trigger("Partisan");
    jet_assert(tr != nullptr);
    jet_assert(tr->event_name.equals("Partisan"));
    tr = ev->find_trigger("Lawrencium");
    jet_assert(tr == nullptr);
    delete(ev);
  }

  if (success) {
    Event* ev = new Event("Halooo", &TestHandler, TRIGGER_MANUAL);
    jet_dbgprint("hub event: reset");
    jet_assert(ev->add_trigger("Partisan"));
    jet_assert(ev->triggers.get(0)->data_ready == false);
    ev->triggers.get(0)->data_ready = true;
    jet_assert(ev->triggers.get(0)->data_ready == true);
    ev->reset_triggers();
    jet_assert(ev->triggers.get(0)->data_ready == false);
    delete(ev);
  }

  if (success) {
    Event* ev = new Event("TestEvent", &TestHandler, TRIGGER_MANUAL);
    if (success) {
      TestHandlerCounter = 0;
      Datum datum = { 0 };
      jet_dbgprint("hub event: take_action");
      jet_assert(ev->add_trigger("TestEventTrigger"));
      ev->triggers.get(0)->data_ready = true;
      jet_assert(!ev->take_action(datum));
      jet_assert(ev->triggers.get(0)->data_ready == false);
      jet_assert(TestHandlerCounter == 1);
    }
    if (success) {
      TestHandlerCounter = 0;
      Datum datum = { 0 };
      ev->reset_triggers();
      jet_dbgprint("hub event: deliver");
      jet_assert(ev->triggers.get(0)->data_ready == false);
      jet_assert(ev->triggers.get(0)->data.uin16 == 0xdead);
      jet_assert(!ev->deliver_trigger("DoesNotExist", datum, datum));
      jet_assert(ev->triggers.get(0)->data_ready == false);
      jet_assert(ev->triggers.get(0)->data.uin16 == 0xdead);
      jet_assert(TestHandlerCounter == 0);
      datum.uin16 = 5;
      jet_assert(!ev->deliver_trigger("TestEventTrigger", datum, datum));
      jet_assert(ev->triggers.get(0)->data_ready == true);
      jet_assert(ev->triggers.get(0)->data.uin16 == 5);
      jet_assert(TestHandlerCounter == 0);
    }
    delete(ev);
  }

  if (success) {
    jet_dbgprint("hub event: trigger firing and conditions");
    Event* ev;
    Datum datum;

    TestHandlerCounter = 0;
    ev = new Event("ManualEvent", &TestHandler, TRIGGER_MANUAL);
    datum = { 0 };
    jet_assert(ev->add_trigger("SingleTrigger"));
    jet_assert(!ev->deliver_trigger("SingleTrigger", datum, datum));
    jet_assert(TestHandlerCounter == 0);
    delete(ev);

    TestHandlerCounter = 0;
    ev = new Event("AnyEvent", &TestHandler, TRIGGER_ON_ANY);
    datum = { 0 };
    jet_assert(ev->add_trigger("TriggerOne"));
    jet_assert(ev->add_trigger("TriggerTwo"));
    jet_assert(!ev->deliver_trigger("TriggerOne", datum, datum));
    jet_assert(TestHandlerCounter == 1);
    jet_assert(!ev->deliver_trigger("TriggerTwo", datum, datum));
    jet_assert(TestHandlerCounter == 2);
    delete(ev);

    TestHandlerCounter = 0;
    ev = new Event("AnyEvent", &TestHandler, TRIGGER_ON_ALL);
    datum = { 0 };
    jet_assert(ev->add_trigger("TriggerOne"));
    jet_assert(ev->add_trigger("TriggerTwo"));
    jet_assert(!ev->deliver_trigger("TriggerOne", datum, datum));
    jet_assert(TestHandlerCounter == 0);
    jet_assert(!ev->deliver_trigger("TriggerTwo", datum, datum));
    jet_assert(TestHandlerCounter == 1);
    delete(ev);

#ifdef JET_EVT_HUB_TEMPORAL
    TestHandlerCounter = 0;
    ev = new Event("TemporalEvent", &TestHandler, TRIGGER_TEMPORAL);
    datum = { 0 };
    jet_assert(ev->add_trigger("SingleTrigger"));
    jet_assert(!ev->deliver_trigger("SingleTrigger", datum, datum));
    jet_assert(TestHandlerCounter == 0);
    ev->take_action(datum);
    jet_assert(TestHandlerCounter == 1);
    delete(ev);
#endif
  }

  if (success) {
    jet_dbgprint("hub: constructor/destructor");
    Hub* hub = new Hub();
    jet_assert(hub != nullptr);
    delete(hub);
  }

  if(success) {
    Hub* hub = new Hub();
    if (success) {
      jet_dbgprint("hub: normal Event insertion");
      TestHandlerCounter = 0;
      jet_assert(hub->add_event("Event1", TestHandler, TRIGGER_ON_ANY));
      jet_assert(hub->m_event_list.size() == 1);
      jet_assert(hub->m_event_list.get(0)->event_id == "Event1");
      jet_assert(hub->m_event_list.get(0)->type == TRIGGER_ON_ANY);
    }
    if (success) {
      jet_dbgprint("hub: trigger insertion");
      jet_assert(hub->add_event_trigger("Event1", "Event1Trigger1"));
      jet_assert(hub->find_event("Event1") != nullptr);
      jet_assert(hub->find_event("Event1")->find_trigger("Event1Trigger1") != nullptr);
    }
    if (success) {
      jet_dbgprint("hub: temporal insertion");
      jet_assert(hub->add_event("Event2", TestHandler, TRIGGER_TEMPORAL, 1000UL));
      jet_assert(hub->find_event("Event2") != nullptr);
    }
    if (success) {
      TestHandlerCounter = 0;
      jet_dbgprint("hub: update");
      hub->update(1000);
      jet_assert(TestHandlerCounter == 1);
      hub->update(2000);
      jet_assert(TestHandlerCounter == 2);
    }
    delete(hub);
  }

  if (success) {
    TestHandlerCounter = 0;
    Hub* hub = new Hub();
    Datum datum = { 0 };
    jet_dbgprint("hub: delivery");
    jet_assert(hub->add_event("OnAny", &TestHandlerProducer, TRIGGER_ON_ANY));
    jet_assert(hub->add_event_trigger("OnAny", "Trigger1"));
    jet_assert(hub->add_event_trigger("OnAny", "Trigger2"));
    jet_assert(hub->add_event("OnAll", &TestHandlerProducer, TRIGGER_ON_ALL));
    jet_assert(hub->add_event_trigger("OnAll", "Trigger3"));
    jet_assert(hub->add_event_trigger("OnAll", "Trigger4"));
    jet_assert(hub->deliver("Trigger1", datum));
    jet_assert(TestHandlerCounter == 1);
    jet_assert(hub->deliver("Trigger2", datum));
    jet_assert(TestHandlerCounter == 2);
    jet_assert(hub->deliver("Trigger3", datum));
    jet_assert(TestHandlerCounter == 2);
    jet_assert(hub->deliver("Trigger4", datum));
    jet_assert(TestHandlerCounter == 3);
    jet_assert(hub->add_event("Tier2", &TestHandlerProducer, TRIGGER_ON_ALL));
    jet_assert(hub->add_event_trigger("Tier2", "OnAll"));
    jet_assert(hub->add_event_trigger("Tier2", "OnAny"));
    jet_assert(hub->deliver("Trigger2", datum));
    jet_assert(TestHandlerCounter == 4);
    jet_assert(hub->deliver("Trigger3", datum));
    jet_assert(TestHandlerCounter == 4);
    jet_assert(hub->deliver("Trigger4", datum));
    jet_assert(TestHandlerCounter == 6);
    delete(hub);
  }

  return success;
}

#endif // JET_TEST

} // namespace evt

} // namespace jet
