
#pragma once

//
// Copyright (C) 2023 Jon Kunkee jonathan.kunkee@gmail.com
// License: BSD 3-clause
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

#define jet_assert_var bool success = true;
#define jet_assert(expr) \
  if (success && !(expr)) { \
    success = false; \
    jet_dbgprint("Assertion failed: '%s' at line %d", #expr, __LINE__); \
  }

#else // JET_TEST
#define jet_dbgprint(...)
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
    jet_dbgprint("insert idx:%d item:%p", idx, item);
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
    jet_dbgprint("append %p", item);
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
    jet_dbgprint("remove %d", idx);
    if (idx == -1) {
      idx = m_count-1;
    }
    for (unsigned int src_idx = idx + 1; src_idx < m_count; src_idx++) {
      int dst_idx = src_idx - 1;
      jet_dbgprint("remove collapse %d<-%d", dst_idx, src_idx);
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

static bool PointerListTest() {
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
  const time_t m_max_interval = (1UL << (sizeof(time_t) * 8 - 1)) - 1;
public:
  DeltaClock() : m_head(nullptr), m_last_update(0) {}
  ~DeltaClock() { clear(); }
  // typically called with millis(), but works with any unsigned monotonic time value
  void update(time_t now) {
    time_t delta = now - m_last_update;
    // monotonic time counter rollover
    if (now < m_last_update) {
      jet_dbgprint("DeltaClock monotonic timer wraparound");
      // type math validated in test suite
      delta = ((unsigned)-(signed)(m_last_update)) + now;
    }
    // Update entries
    while (m_head != NULL) {
      if (m_head->remaining > delta) {
        m_head->remaining -= delta;
        break;
      } else {
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
      return false;
    }
    // Initialize
    new_entry->remaining = new_entry->interval;
    // Insert
    // Empty list
    if (m_head == nullptr) {
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
        new_entry->next = next;
        m_head = new_entry;
        next->remaining -= new_entry->remaining;
        break;
      } else if (next == NULL) {
        // end
        prev->next = new_entry;
        break;
      } else if (new_entry->remaining < next->remaining) {
        // middle
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
#ifdef JET_TEST
  friend bool DeltaClockTest();
#endif // JET_TEST
};

#ifdef JET_TEST

#define COUNTER_ENTRY(id, ivl, rep) \
static int Counter##id; \
static void Action##id(void*) { \
  Counter##id++; \
} \
static DeltaClock::Entry Entry##id = { \
  .action = &Action##id, \
  .context = nullptr, \
  .interval = ivl, \
  .repeating = rep, \
};

COUNTER_ENTRY(A, 1000, false)
COUNTER_ENTRY(B, 2000, false)

COUNTER_ENTRY(1, 1000, false);
COUNTER_ENTRY(2, 10000, false);
COUNTER_ENTRY(3, 1000, false);
COUNTER_ENTRY(4, 1000, false);
COUNTER_ENTRY(5, 1000, false);
COUNTER_ENTRY(6, 12*60*60*1000, false);
COUNTER_ENTRY(7, 1200, false);

static bool DeltaClockTest() {
  jet_assert_var;
  DeltaClock* clock;

  if (success) {
    jet_dbgprint("time type properties");
    time_t big_time = -1000;
    time_t small_time = 1000;
    jet_assert(big_time > small_time);
    jet_assert(((unsigned)-(signed)(big_time)) + small_time == 2000);
    // My old method does not work the way it was
    //success = success && (time_t)(-1) - big_time + small_time == 2000;
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
  delete(clock);

  return success;
}

#endif // JET_TEST


#ifdef JET_TEST

static bool HubTest() {
  return false;
}

#endif // JET_TEST

} // namespace evt

} // namespace jet
