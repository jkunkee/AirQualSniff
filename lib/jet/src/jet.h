
#pragma once

//
// Copyright (C) 2023 Jon Kunkee jonathan.kunkee@gmail.com
// License: BSD 3-clause
//

#define JET_TEST

#ifdef JET_TEST
#ifdef PARTICLE_WIRING_ARDUINO_COMPATIBILTY
#define dbgprint(a, ...) Serial.printlnf(__VA_ARGS__)
#elif defined(ARDUINO)
// https://github.com/arduino-libraries/Arduino_DebugUtils/blob/master/src/Arduino_DebugUtils.h
#include <Arduino_DebugUtils.h>
Arduino_DebugUtils eventhub_arduino_dbg;
#define dbgprint(...) eventhub_arduino_dbg.print(0, __VA_ARGS__)
#endif
#else // JET_TEST
#define dbgprint(...)
#endif // JET_TEST

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
    m_list = malloc(sizeof(T*)*initial_capacity);
    m_capacity = initial_capacity;
    if (m_list == nullptr) {
      m_capacity = 0;
    }
  }
  ~PointerList()
  {
    if (m_list_is_internally_managed) {
      free(m_list);
    }
  }
  bool insert(int idx, T* item)
  {
    dbgprint("insert idx:%d item:%p", idx, item);
    if (item == nullptr) {
      dbgprint("insert null check failed");
      return false;
    }
    // Insertion can land at the end (-1) or at any existing index
    if (idx == -1) {
      idx = m_count;
    }
    if (idx < 0 || (signed)m_count < idx) {
      dbgprint("insert range check failed");
      return false;
    }
    // Is there room?
    if (m_count + 1 > m_capacity) {
      if (m_list_is_internally_managed) {
      dbgprint("insert increase needed");
        // Allocate a bigger buffer.
        set_capacity(m_capacity + 2);
      } else {
        dbgprint("insert size check failed");
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
    dbgprint("append %p", item);
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
    if (idx < 0 || m_count <= idx) {
      return nullptr;
    }
    return m_list[idx];
  }
  bool remove(int idx) {
    if (idx < -1 || (signed)m_count <= idx) {
      dbgprint("remove failed range check %d", idx);
      return false;
    }
    dbgprint("remove %d", idx);
    if (idx == -1) {
      idx = m_count-1;
    }
    for (int src_idx = idx + 1; src_idx < m_count; src_idx++) {
      int dst_idx = src_idx - 1;
      dbgprint("remove collapse %d<-%d", dst_idx, src_idx);
      m_list[dst_idx] = m_list[src_idx];
    }
    m_count -= 1;
    return true;
  }
  int find_first(T* item) {
    for (int idx = 0; idx < m_count; idx++) {
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
    T** new_buf = malloc(sizeof(T*) * new_capacity);
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
  bool success = true;
  PointerList<int>* list;

  int mem[4];
  int* ptrs[4];

  list = new PointerList<int>(ptrs, 4);
  success = success && list->m_capacity == 4;
  success = success && list->m_list == ptrs;
  success = success && list->m_list_is_internally_managed == false;
  success = success && list->m_count == 0;
  success = success && list->append(&mem[0]);
  success = success && list->m_count == 1;
  success = success && list->append(&mem[1]);
  success = success && list->m_count == 2;
  success = success && list->append(&mem[2]);
  success = success && list->m_count == 3;
  success = success && list->append(&mem[3]);
  success = success && list->m_count == 4;
  success = success && !list->append(&mem[0]);
  success = success && list->m_count == 4;
  delete(list);

  list = new PointerList<int>(4);
  success = success && list->m_capacity == 4;
  success = success && list->m_list != nullptr;
  success = success && list->m_list_is_internally_managed == true;
  success = success && list->is_empty();
  success = success && list->size() == 0;
  success = success && list->append(&mem[0]);
  success = success && list->size() == 1;
  success = success && list->m_list[0] == &mem[0];
  success = success && list->append(&mem[1]);
  success = success && list->size() == 2;
  success = success && list->m_list[1] == &mem[1];
  success = success && list->append(&mem[2]);
  success = success && list->size() == 3;
  success = success && list->append(&mem[3]);
  success = success && list->size() == 4;
  success = success && list->capacity() == 4;
  success = success && list->is_full();
  success = success && list->append(&mem[0]);
  success = success && list->size() == 5;
  success = success && list->capacity() == 4 + 2;
  list->clear();
  success = success && list->size() == 0;

  success = success && list->capacity() == 4 + 2;
  list->append(&mem[0]);
  list->append(&mem[1]);
  success = success && list->m_list[0] == &mem[0];
  success = success && list->m_list[1] == &mem[1];
  success = success && list->swap(0, 1);
  success = success && list->m_list[0] == &mem[1];
  success = success && list->m_list[1] == &mem[0];
  success = success && list->remove(0);
  success = success && list->size() == 1;
  success = success && list->m_list[0] == &mem[0];
  success = success && list->remove(0);
  success = success && list->size() == 0;

  success = success && list->insert(0, &mem[0]);
  success = success && list->m_list[0] == &mem[0];
  success = success && list->insert(0, &mem[1]);
  success = success && list->m_list[0] == &mem[1];
  success = success && list->m_list[1] == &mem[0];
  success = success && list->insert(1, &mem[2]);
  success = success && list->m_list[0] == &mem[1];
  success = success && list->m_list[1] == &mem[2];
  success = success && list->m_list[2] == &mem[0];
  success = success && list->get(0) == &mem[1];
  success = success && list->get(1) == &mem[2];
  success = success && list->get(2) == &mem[0];
  success = success && list->get(3) == nullptr;
  list->clear();

  list->insert(-1, &mem[0]);
  list->insert(-1, &mem[1]);
  list->insert(-1, &mem[2]);
  list->insert(-1, &mem[3]);
  success = success && list->find_first(&mem[2]) == 2;
  success = success && list->remove_first(&mem[2]);
  success = success && list->find_first(&mem[2]) == -1;
  success = success && list->find_first(&mem[3]) == 2;
  success = success && !list->remove_first(&mem[2]);
  success = success && list->find_first(&mem[3]) == 2;
  success = success && list->size() == 3;
  delete(list);

  if (success) {
    dbgprint("Success!");
  } else {
    dbgprint("Failure...");
  }

  return success;
}

#endif

} // namespace jet
