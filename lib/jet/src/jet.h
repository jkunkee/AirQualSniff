
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

#ifndef PRIu32
#error Standard library does not provide modular decimal unsigned integer print format specifier
#endif

#ifdef PARTICLE_WIRING_ARDUINO_COMPATIBILTY
#undef F
#define F(X) (X)
#define jet_dbgprint(...) Serial.printlnf(__VA_ARGS__)
#elif defined(ARDUINO)
// https://github.com/arduino-libraries/Arduino_DebugUtils/blob/master/src/Arduino_DebugUtils.h
#include <Arduino_DebugUtils.h>
Arduino_DebugUtils eventhub_arduino_dbg;
#define jet_dbgprint(...) eventhub_arduino_dbg.print(DBG_ERROR, __VA_ARGS__)
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
    jet_dbgprint(F("Assertion failed: '%s' at line %d"), #expr, __LINE__); \
  }

#else // JET_TEST
#define jet_dbgprint(...)
#define jet_traceprint(...)
#endif // JET_TEST

// JET's temporal code was designed to be told when it is and not have to ask.
// While this is perfectly functional, it makes certain kinds of debugging
// impossible. If you #define JET_EVT_MEASURE_TIMING_FUNC to be a function that
// takes no arguments and returns a uint32_t -- think Arduino's millis() --
// then various kinds of temporal debugging information will be tracked and can
// be printed with debug_string functions.
//
// JET_EVT_MEASURE_TIMING_FUNC: uint32_t get_time(void)

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
      jet_dbgprint(F("insert null check failed"));
      return false;
    }
    // Insertion can land at the end (-1) or at any existing index
    if (idx == -1) {
      idx = m_count;
    }
    if (idx < 0 || (signed)m_count < idx) {
      jet_dbgprint(F("insert range check failed"));
      return false;
    }
    // Is there room?
    if (m_count + 1 > m_capacity) {
      if (m_list_is_internally_managed) {
        jet_dbgprint(F("insert increase needed"));
        // Allocate a bigger buffer.
        set_capacity(m_capacity + 2);
      } else {
        jet_dbgprint(F("insert size check failed"));
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
      jet_dbgprint(F("remove failed range check %d"), idx);
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
    if (idx1 < 0 || (signed)m_count <= idx1 || idx2 < 0 || (signed)m_count <= idx2) {
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
    jet_dbgprint(F("Success!"));
  } else {
    jet_dbgprint(F("Failure..."));
  }

  return success;
}

#endif // JET_TEST

// UInt32 class
// Provides a two's compliment 32-bit unsigned integer type with predictable rollover
// and useful operation flags.
// Since 8-bit microcontrollers have to use combinations of instructions to do larger
// type arithmetic, they don't always have predictable overflow and type conversion
// characteristics.

class UInt32 {
private:
  uint8_t m_bytes[4];
  bool m_overflow;
  bool m_underflow;
  void clear_flags() {
    m_overflow = false;
    m_underflow = false;
  }
  uint32_t to_uint32() const {
    uint32_t result = 0;
    result += m_bytes[3] << 24;
    result += m_bytes[2] << 16;
    result += m_bytes[1] << 8;
    result += m_bytes[0] << 0;
    return result;
  }
  void from_uint32(uint32_t in) {
    m_bytes[3] = (in >> 24) & 0xFF;
    m_bytes[2] = (in >> 16) & 0xFF;
    m_bytes[1] = (in >> 8) & 0xFF;
    m_bytes[0] = (in >> 0) & 0xFF;
  }
public:
  static constexpr uint32_t MAX = UINT32_MAX;
  UInt32() {
    m_bytes[0] = 0;
    m_bytes[1] = 0;
    m_bytes[2] = 0;
    m_bytes[3] = 0;
    clear_flags();
  }
  UInt32(const UInt32 &in) {
    clear_flags();
    m_bytes[0] = in.m_bytes[0];
    m_bytes[1] = in.m_bytes[1];
    m_bytes[2] = in.m_bytes[2];
    m_bytes[3] = in.m_bytes[3];
  }

  UInt32(uint32_t in) { this->operator=(in); }
  UInt32(uint16_t in) { this->operator=(in); }
  UInt32(uint8_t in) { this->operator=(in); }
  UInt32(int32_t in) { this->operator=(in); }
  UInt32(int16_t in) { this->operator=(in); }
  UInt32(int8_t in) { this->operator=(in); }
  //UInt32(int in) { this->operator=((int32_t)in); }

  // Reference: https://en.wikipedia.org/wiki/Operators_in_C_and_C%2B%2B

  UInt32 & operator = (uint32_t in) {
    clear_flags();
    from_uint32(in);
    return *this;
  }
  UInt32 & operator = (uint16_t in) { this->operator=((uint32_t)in); return *this; }
  UInt32 & operator = (uint8_t in) { this->operator=((uint32_t)in); return *this; }
  UInt32 & operator = (int32_t in) {
    clear_flags();
    if (in >= 0) {
      from_uint32((uint32_t)in);
    } else {
      jet_traceprint(F("jet::UInt32 assignment flushing %d to 0"), in);
      from_uint32(0);
      m_underflow = true;
    }
    return *this;
  }
  UInt32 & operator = (int16_t in) { this->operator=((int32_t)in); return *this; }
  UInt32 & operator = (int8_t in) { this->operator=((int32_t)in); return *this; }

  // Addition
  friend UInt32 operator + (const UInt32 &lhs, const UInt32 &rhs);
  UInt32 & operator += (const UInt32 &in) { this->operator=(*this + in); return *this; }
  UInt32 & operator ++ () { this->operator=(*this + (uint32_t)1); return *this; }

  // Subtraction
  friend UInt32 operator - (const UInt32 &lhs, const UInt32 &rhs);
  UInt32 & operator -= (const UInt32 &in) { this->operator=(*this - in); return *this; }
  UInt32 & operator -- () { this->operator=(*this - (uint32_t)1); return *this; }
  // Multiplication
  friend UInt32 operator * (const UInt32 &lhs, const UInt32 &rhs);
  // Division
  friend UInt32 operator / (const UInt32 &lhs, const UInt32 &rhs);
  // Modulo?
  // Comparisons
  // all other variants call operator > and so don't need friendship
  friend bool operator > (const UInt32 &lhs, const UInt32 &rhs);
  friend bool operator == (const UInt32 &lhs, const UInt32 &rhs);
  // Typecast
  //operator uint32_t() { return this->to_uint32(); } // this makes 2 + a ambiguous because it can be coerced back
  uint32_t to_uint32_t() { return this->to_uint32(); }
  operator String() {
    String out;
    debug_string(out);
    return out;
  }

  friend bool UInt32Test();

  void debug_string(String& out) const {
    out += "s0x";
    if (m_bytes[3] < 128) { out += "0"; }
    out += String(m_bytes[3], (uint8_t)16);
    if (m_bytes[2] < 128) { out += "0"; }
    out += String(m_bytes[2], (uint8_t)16);
    if (m_bytes[1] < 128) { out += "0"; }
    out += String(m_bytes[1], (uint8_t)16);
    if (m_bytes[0] < 128) { out += "0"; }
    out += String(m_bytes[0], (uint8_t)16);
  }
};

// Left-hand operand type coersion is only possible in non-member functions; see Arduino's String.h and
// https://stackoverflow.com/questions/4652932/why-define-operator-or-outside-a-class-and-how-to-do-it-properly
// Also, member binary operators confuse me about which operand is where.

inline UInt32 operator + (const UInt32 &lhs, const UInt32 &rhs) {
  uint16_t z, l, r;
  bool carry;
  UInt32 out;

  out.clear_flags();

  l = (uint16_t)lhs.m_bytes[0];
  r = (uint16_t)rhs.m_bytes[0];
  z = l + r;
  out.m_bytes[0] = (uint8_t)z;
  carry = z > (uint16_t)UINT8_MAX;

  l = (uint16_t)lhs.m_bytes[1];
  r = (uint16_t)rhs.m_bytes[1];
  z = l + r;
  if (carry) { z += 1; }
  out.m_bytes[1] = (uint8_t)z;
  carry = z > (uint16_t)UINT8_MAX;

  l = (uint16_t)lhs.m_bytes[2];
  r = (uint16_t)rhs.m_bytes[2];
  z = l + r;
  if (carry) { z += 1; }
  out.m_bytes[2] = (uint8_t)z;
  carry = z > (uint16_t)UINT8_MAX;

  l = (uint16_t)lhs.m_bytes[3];
  r = (uint16_t)rhs.m_bytes[3];
  z = l + r;
  if (carry) { z += 1; }
  out.m_bytes[3] = (uint8_t)z;
  carry = z > (uint16_t)UINT8_MAX;

  out.m_overflow = carry;

  jet_traceprint("%" PRIx32 " + %" PRIx32 " => %" PRIx32, lhs.to_uint32(), rhs.to_uint32(), out.to_uint32());

  return out;
}

inline bool operator > (const UInt32 &lhs, const UInt32 &rhs) {
  // Yes, this can be simplified. A lot. But it works, and it's done.
  if (lhs.m_bytes[3] > rhs.m_bytes[3]) {
    return true;
  } else if (lhs.m_bytes[3] == rhs.m_bytes[3] && lhs.m_bytes[2] > rhs.m_bytes[2]) {
    return true;
  } else if (lhs.m_bytes[3] == rhs.m_bytes[3] && lhs.m_bytes[2] == rhs.m_bytes[2] && lhs.m_bytes[1] > rhs.m_bytes[1]) {
    return true;
  } else if (lhs.m_bytes[3] == rhs.m_bytes[3] && lhs.m_bytes[2] == rhs.m_bytes[2] && lhs.m_bytes[1] == rhs.m_bytes[1] && lhs.m_bytes[0] > rhs.m_bytes[0]) {
    return true;
  } else {
    return false;
  }
}
inline bool operator >= (const UInt32 &lhs, const UInt32 &rhs) { return !(rhs > lhs); }
inline bool operator < (const UInt32 &lhs, const UInt32 &rhs) { return (rhs > lhs); }
inline bool operator <= (const UInt32 &lhs, const UInt32 &rhs) { return !(lhs > rhs); }
inline bool operator == (const UInt32 &lhs, const UInt32 &rhs) {
  return lhs.m_bytes[3] == rhs.m_bytes[3] &&
         lhs.m_bytes[2] == rhs.m_bytes[2] &&
         lhs.m_bytes[1] == rhs.m_bytes[1] &&
         lhs.m_bytes[0] == rhs.m_bytes[0];
}

inline UInt32 operator - (const UInt32 &lhs, const UInt32 &rhs) {
  UInt32 inv;
  // Two's complement
  inv.m_bytes[0] = ~rhs.m_bytes[0];
  inv.m_bytes[1] = ~rhs.m_bytes[1];
  inv.m_bytes[2] = ~rhs.m_bytes[2];
  inv.m_bytes[3] = ~rhs.m_bytes[3];
  inv += (uint32_t)1;
  // Outsource math to addition operator
  UInt32 out = lhs + inv;
  out.m_overflow = false;
  if (lhs < rhs) {
    out.m_underflow = true;
  }
  return out;
}

UInt32 operator * (const UInt32 &lhs, const UInt32 &rhs) {
  // cascading multiply
//  uint8_t out[9] = { 0 };
//
//  for (int lcol = 0; lcol < 4; lcol++) {
//    for (int rcol = 0; rcol < 4; rcol++) {
//      uint16_t l = (uint16_t)lhs.m_bytes[lcol];
//      uint16_t r = (uint16_t)rhs.m_bytes[rcol];
//      uint16_t z = l * r;
//      uint16_t lowsum = (uint16_t)out[lcol + rcol] + (z & 0xFF);
//      out[lcol + rcol] = (uint8_t)lowsum;
//      uint16_t highsum = (uint16_t)out[lcol + rcol + 1] + ((z >> 8) & 0xFF) + (lowsum >> 8);
//      out[lcol + rcol + 1] += (uint8_t)highsum;
//      out[lcol + rcol + 2] += 0; // CASCADE INCOMPLETE
//    }
//  }
//
  UInt32 result;
  // operate on bigger type, then downcast
//  result.m_bytes[0] = out[0];
//  result.m_bytes[1] = out[1];
//  result.m_bytes[2] = out[2];
//  result.m_bytes[3] = out[3];
//  result.m_overflow = out[4] != 0 ||
//                      out[5] != 0 ||
//                      out[6] != 0 ||
//                      out[7] != 0;
//  uint64_t lbig = lhs.to_uint32();
//  uint64_t rbig = rhs.to_uint32();
//  uint64_t out = lbig * rbig;
//  result.from_uint32((uint32_t)out);
//  result.m_overflow = (out >> 32) != 0;
  // TODO
  result = lhs.to_uint32() * rhs.to_uint32(); // is my to_uint32 broken?
  return result;
}
UInt32 operator / (const UInt32 &lhs, const UInt32 &rhs) {
  // TODO
  return lhs.to_uint32() / rhs.to_uint32();
}

#ifdef JET_TEST

bool UInt32Test() {
  jet_assert_var(success);

  if (success) {
    jet_dbgprint(F("basic constructor and uint assignment"));
    UInt32 obj;

    jet_assert(obj.m_bytes[0] == 0);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    obj = (uint32_t)1;

    jet_assert(obj.m_bytes[0] == 1);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);

    obj = (uint16_t)1;

    jet_assert(obj.m_bytes[0] == 1);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);

    obj = (uint8_t)1;

    jet_assert(obj.m_bytes[0] == 1);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);

    if (!success) {
      String out;
      obj.debug_string(out);
      jet_dbgprint(out.c_str());
    }
  }

  if (success) {
    jet_dbgprint(F("signed assignment"));
    UInt32 obj;

    obj = (int32_t)1;

    jet_assert(obj.m_bytes[0] == 1);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    obj = (int32_t)INT32_MAX;

    jet_assert(obj.m_bytes[0] == ((INT32_MAX >> 0) & 0xFF));
    jet_assert(obj.m_bytes[1] == ((INT32_MAX >> 8) & 0xFF));
    jet_assert(obj.m_bytes[2] == ((INT32_MAX >> 16) & 0xFF));
    jet_assert(obj.m_bytes[3] == ((INT32_MAX >> 24) & 0xFF));
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    obj = (int32_t)INT32_MIN;

    jet_assert(obj.m_bytes[0] == 0);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == true);

    // reset so changes are visible
    obj = (uint32_t)0;

    obj = (int16_t)1;

    jet_assert(obj.m_bytes[0] == 1);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    obj = (int16_t)INT16_MIN;

    jet_assert(obj.m_bytes[0] == 0);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == true);

    // reset so changes are visible
    obj = (uint32_t)0;

    obj = (int8_t)1;

    jet_assert(obj.m_bytes[0] == 1);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    obj = (int8_t)INT8_MIN;

    jet_assert(obj.m_bytes[0] == 0);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == true);

    if (!success) {
      String out;
      obj.debug_string(out);
      jet_dbgprint(out.c_str());
    }
  }

  if (success) {
    jet_dbgprint(F("operator +"));
    UInt32 obj, a, b;

    obj = 0;
    a = 1;
    b = 0;

    obj = a + b;

    jet_assert(obj.m_bytes[0] == 1);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    obj = 0;
    a = (uint32_t)0xaaaaaaaa;
    b = (uint32_t)0x55555555;

    obj = a + b;

    jet_assert(obj.m_bytes[0] == 0xFF);
    jet_assert(obj.m_bytes[1] == 0xFF);
    jet_assert(obj.m_bytes[2] == 0xFF);
    jet_assert(obj.m_bytes[3] == 0xFF);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    obj = 0;
    a = (uint32_t)257;
    b = (uint32_t)UINT32_MAX;

    obj = a + b;

    jet_assert(obj.m_bytes[0] == 0);
    jet_assert(obj.m_bytes[1] == 1);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == true);
    jet_assert(obj.m_underflow == false);

    obj = 0;
    a = (int32_t)-16; // flushes to zero
    b = (uint32_t)32;

    obj = a + b;

    jet_assert(obj.m_bytes[0] == 32);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    obj = 0;
    a = 1;

    obj = a + 2;

    jet_assert(obj.m_bytes[0] == 3);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    obj = 0;
    a = 1;

    obj = 2 + a;

    jet_assert(obj.m_bytes[0] == 3);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    if (!success) {
      String out;
      obj.debug_string(out);
      jet_dbgprint(out.c_str());
    }
  }

  if (success) {
    jet_dbgprint(F("operator +="));
    UInt32 obj, a;

    obj = 0;
    a = 1;

    obj += a;

    jet_assert(obj.m_bytes[0] == 1);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    obj = 0;

    obj += (uint32_t)1;

    jet_assert(obj.m_bytes[0] == 1);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    obj = 14;

    obj += (int16_t)1;

    jet_assert(obj.m_bytes[0] == 15);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    if (!success) {
      String out;
      obj.debug_string(out);
      jet_dbgprint(out.c_str());
    }
  }

  if (success) {
    jet_dbgprint(F("operators < > <= >= =="));
    UInt32 a, b;

    a = 1;
    b = 2;

    jet_assert(b > a == true);
    jet_assert(a > b == false)
    jet_assert(a < b == true);
    jet_assert(b < a == false);
    jet_assert(b <= b == true);
    jet_assert(a <= b == true);
    jet_assert(b <= a == false);
    jet_assert(a >= a == true);
    jet_assert(b >= a == true);
    jet_assert(a >= b == false);
  }

  if (success) {
    jet_dbgprint(F("operator -"));
    UInt32 obj, a, b;

    obj = 0;
    a = 2;
    b = 1;

    obj = a - b;

    jet_assert(obj.m_bytes[0] == 1);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    if (!success) {
      String out;
      obj.debug_string(out);
      jet_dbgprint(out.c_str());
    }
  }

  if (success) {
    jet_dbgprint(F("operator *"));
    UInt32 obj, a, b;

    obj = 0;
    a = 3;
    b = 5;

    obj = a * b;

    jet_assert(obj.m_bytes[0] == 15);
    jet_assert(obj.m_bytes[1] == 0);
    jet_assert(obj.m_bytes[2] == 0);
    jet_assert(obj.m_bytes[3] == 0);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    obj = 0;
    a = 65535;
    b = 999;

    obj = a * b;

    jet_assert(obj.m_bytes[0] == 0x19);
    jet_assert(obj.m_bytes[1] == 0xfc);
    jet_assert(obj.m_bytes[2] == 0xe6);
    jet_assert(obj.m_bytes[3] == 0x3);
    jet_assert(obj.m_overflow == false);
    jet_assert(obj.m_underflow == false);

    if (!success) {
      String out;
      obj.debug_string(out);
      jet_dbgprint(out.c_str());
    }
  }

  return success;
}

#endif // JET_TEST


namespace evt {

// time_t type handling is inconsistent across compilers and types.
// Arduino treated 1000 and 1000UL as so drastically different the code failed to work.
// Particle somehow passes a computed max integer value as 0.
//
// So, we use types that are easy to reason about.
//
// uint32_t goes 0 to 4,294,967,295.
//
// If we use the top half of the range for rollover detection--that is, we assume that
// the largest input time delta and largest event interval is UINT32_MAX/2--and then
// we assume a millisecond time unit, we can represent time spans as large as
// 4,294,967,295 / 2 => 2147483647 ms
// 2147483647 / 1000 => 2147483 s
// 2147483 / 60 => 35791 min
// 35791 / 60 => 596 h
// 596 / 24 => 24 d
//
// This seems like a good maximum for an embedded timer library. If a design approaches
// these time scales, it probably also needs wall-clock and calendar synchronization
// which this library does not have.

typedef UInt32 jet_time_t;

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
    jet_time_t interval;
    boolean repeating;
    jet_time_t remaining;
    Entry* next;
  };
private:
  Entry* m_head;
  jet_time_t m_last_update;
  bool m_first_update = true;
  // constant for detecting most rollover, overflow, and underflow conditions
  // max type value / 2
  //const jet_time_t m_max_interval = ((jet_time_t)0xffffffffffffffffULL) >> 1;
  const jet_time_t m_max_interval = UInt32::MAX / 2;
public:
  DeltaClock() : m_head(nullptr), m_last_update((uint32_t)0) {}
  ~DeltaClock() { clear(); }
  void first_update(jet_time_t first_now) {
  }
  // typically called with millis(), but works with any unsigned monotonic time value
  void update(jet_time_t now) {
    if (m_first_update) {
      m_last_update = now;
      m_first_update = false;
      return;
    }
    // Constrain delta to be nonzero
    if (now == m_last_update) {
      jet_dbgprint(F("no time has passed (or exactly one max-jet_time_t time interval has passed)"));
      return;
    }
    jet_time_t delta = now - m_last_update;
    // monotonic time counter rollover
    if (now < m_last_update) {
      delta = UInt32::MAX - m_last_update + now + (uint32_t)1 /* correction for JET_TIME_T_MAX == 2^bits - 1 */;
      jet_dbgprint(F("DeltaClock monotonic timer wraparound %" PRIu32 " to %" PRIu32 " is %" PRIu32), m_last_update, now, delta);
    }
    if (delta > (uint32_t)30*(uint32_t)1000) {
      jet_dbgprint("DeltaClock big delta: %" PRIu32, delta);
    }
    jet_traceprint("update from %" PRIu32, m_last_update);
    jet_traceprint("       to %" PRIu32, now);
    jet_traceprint("       delta %" PRIu32, delta);
    jet_traceprint("       head %p", m_head);
    // Update entries
    while (m_head != NULL) {
      jet_traceprint("  delta:%" PRIu32 " ---------------", delta);
      jet_traceprint("  processing %p", m_head);
      jet_traceprint("  remaining:%" PRIu32, m_head->remaining);
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
        entry->remaining = (uint32_t)0;
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
    if (new_entry == nullptr) {
      jet_dbgprint(F("schedule failed with invalid arg: new_entry == nullptr"));
      return false;
    }
    if (new_entry->interval == (uint32_t)0) {
      jet_dbgprint(F("schedule failed with invalid arg: new_entry->interval == 0"));
      return false;
    }
    if (new_entry->interval > m_max_interval) {
      jet_dbgprint(F("schedule failed with invalid arg: new_entry->interval==%" PRIu32 " > max_interval==%" PRIu32), new_entry->interval, m_max_interval);
      return false;
    }
    if (new_entry->action == nullptr) {
      jet_dbgprint(F("schedule failed with invalid arg: new_entry->action == nullptr"));
      return false;
    }
    if (new_entry->next != nullptr) {
      jet_dbgprint(F("schedule failed with invalid arg: new_entry->next==%p != nullptr"), new_entry->next);
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
  //jet_time_t get_remaining_time(DeltaClockEntry* entry);
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
      out += String(entry->remaining);
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
static jet_time_t Counter##id; \
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
//  jet_dbgprint(F("  "#id":%d"), Counter##id);
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
    jet_dbgprint(F("time type properties"));
    jet_time_t big_time;
    big_time -= 1000;
    jet_time_t small_time = 1000;
    jet_assert(sizeof(jet_time_t) >= 4);
    jet_assert(big_time > small_time);
    jet_assert(((unsigned)-(signed)(big_time.to_uint32_t())) + small_time == 2000); // TODO rework with new class
    // My old method does not work the way it was
    //jet_assert((jet_time_t)(-1) - big_time + small_time == 2000);
  }

  clock = new DeltaClock();
  if (success) {
    jet_dbgprint(F("constructor"));
    jet_assert(clock->m_head == nullptr);
    jet_assert(clock->m_last_update == 0);
    jet_assert(clock->m_first_update == true);
  }
  if (success) {
    jet_dbgprint(F("empty list update"));
    clock->update(5000);
    jet_assert(clock->m_last_update == 5000);
  }
  if (success) {
    jet_dbgprint(F("scheduling one event"));
    jet_assert(clock->schedule(&EntryA));
    jet_assert(clock->m_head == &EntryA);
    jet_assert(clock->m_head->remaining == EntryA.interval);
    jet_assert(clock->m_head->next == nullptr);
  }
  if (success) {
    jet_dbgprint(F("schedule a second, following event"));
    jet_assert(clock->schedule(&EntryB));
    jet_assert(clock->m_head == &EntryA);
    jet_assert(clock->m_head->remaining == EntryA.interval);
    jet_assert(clock->m_head->next == &EntryB);
    jet_assert(clock->m_head->next->remaining == EntryB.interval - EntryA.interval);
  }
  if (success) {
    jet_dbgprint(F("clear the list, including next pointers"));
    clock->clear();
    jet_assert(clock->m_head == nullptr);
    jet_assert(EntryA.next == nullptr);
    jet_assert(EntryB.next == nullptr);
  }
  if (success) {
    jet_dbgprint(F("schedule a second, earlier event"));
    jet_assert(clock->schedule(&EntryB));
    jet_assert(clock->m_head == &EntryB);
    jet_assert(clock->m_head->remaining == EntryB.interval);
    jet_assert(clock->schedule(&EntryA));
    jet_assert(clock->m_head == &EntryA);
    jet_assert(clock->m_head->remaining == EntryA.interval);
    jet_assert(clock->m_head->next == &EntryB);
    jet_assert(clock->m_head->next->remaining == EntryB.interval - EntryA.interval);
  }
  if (success) { jet_dbgprint(F("destructor")); }
  delete(clock);
  jet_assert(EntryA.next == nullptr);
  jet_assert(EntryB.next == nullptr);

  clock = new DeltaClock();
  if (success) {
    jet_dbgprint(F("fire both"));
    jet_assert(clock->schedule(&EntryA));
    jet_assert(clock->schedule(&EntryB));
    clock->update(0);
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
    jet_dbgprint(F("repeater requeue"));
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
    jet_dbgprint(F("repeater repeats"));
    clock->update(15000);
    jet_assert(CounterA == 15);
    jet_assert(CounterB == 7);
    jet_assert(clock->m_head == &EntryB);
    jet_assert(EntryA.next == nullptr);
    jet_assert(EntryB.next == &EntryA);
  }
  if (success) {
    jet_dbgprint(F("simultaneous events"));
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
    jet_dbgprint(F("partial repeater costing"));
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
    jet_dbgprint(F("timer overflow"));
    clock->clear();
    clock->m_last_update = 0;
    CounterA = 0;
    EntryA.interval = 1000;
    EntryA.repeating = true;
    clock->update(UINT32_MAX-500);
    jet_assert(clock->schedule(&EntryA));
    clock->update(500-1);
    jet_assert(CounterA == 1);
    jet_assert(EntryA.remaining == 1000);
  }
  if (success) {
    jet_dbgprint(F("periodically simultaneous events"));
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
    jet_dbgprint(F("AirQualSniff Scenario"));
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
    jet_time_t test_duration = 36UL*60UL*60UL*1000UL;
    clock->update(test_duration);
    jet_assert(Counter1 == (test_duration / Entry1.interval));
    jet_assert(Counter2 == (test_duration / Entry2.interval));
    jet_assert(Counter3 == (test_duration / Entry3.interval));
    jet_assert(Counter4 == (test_duration / Entry4.interval));
    jet_assert(Counter5 == (test_duration / Entry5.interval));
    jet_assert(Counter6 == (test_duration / Entry6.interval));
    jet_assert(Counter7 == (test_duration / Entry7.interval));
    if (!success) {
      jet_dbgprint(F("Counter1: %" PRIu32 " / %" PRIu32), Counter1, test_duration / Entry1.interval);
      jet_dbgprint(F("Counter2: %" PRIu32 " / %" PRIu32), Counter2, test_duration / Entry2.interval);
      jet_dbgprint(F("Counter3: %" PRIu32 " / %" PRIu32), Counter3, test_duration / Entry3.interval);
      jet_dbgprint(F("Counter4: %" PRIu32 " / %" PRIu32), Counter4, test_duration / Entry4.interval);
      jet_dbgprint(F("Counter5: %" PRIu32 " / %" PRIu32), Counter5, test_duration / Entry5.interval);
      jet_dbgprint(F("Counter6: %" PRIu32 " / %" PRIu32), Counter6, test_duration / Entry6.interval);
      jet_dbgprint(F("Counter7: %" PRIu32 " / %" PRIu32), Counter7, test_duration / Entry7.interval);
    }
  }
  if (success) {
    jet_dbgprint(F("Sus high-value issues"));
    clock->clear();
    jet_time_t base_time = 0x40000000;
    clock->update(base_time);
    CounterA = 0;
    EntryA.interval = 0x01000000;
    EntryA.repeating = true;
    jet_assert(clock->schedule(&EntryA));
    clock->update(base_time + EntryA.interval);
    jet_assert(CounterA == 1);
    clock->update(base_time + EntryA.interval*16);
    jet_assert(CounterA == 16);
    clock->update(base_time + EntryA.interval*32);
    jet_assert(CounterA == 32);
    clock->update(base_time + EntryA.interval*128);
    jet_assert(CounterA == 128);
    // Integer overflow is Undefined Behavior in C, and the Arduino and Particle compiler behavior
    // turns out to not actually be plain base-2 rollover. Arduino simply drops the high bits, probably
    // as an artifact of the cascaded-register multi-byte add algorithm.
    clock->update(base_time + EntryA.interval*256);
    jet_assert(CounterA == 256);
    clock->update(0);
    jet_assert(CounterA == 0xc0000000/*amt to trigger rollover*/ / 0x01000000/*interval*/);
  }
  if (!success) {
    String str;
    clock->debug_string(str);
    jet_dbgprint(F("%s"), str.c_str());
  }
  delete(clock);

  if (!success) {
    jet_dbgprint(F("EntryA: @%p CounterA:%d"), &EntryA, CounterA);
    jet_dbgprint(F("EntryB: @%p CounterB:%d"), &EntryB, CounterB);
    jet_dbgprint(F("EntryC: @%p CounterC:%d"), &EntryC, CounterC);
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
  void debug_string(String& out) {
    out += F("    ");
    out += event_name;
    out += F(": ");
    out += data_ready;
    if (data_ready) {
      out += " (";
      out += String(data.in16);
      out += ")";
    }
    out += "\n";
  }
#ifdef JET_TEST
  friend bool HubTest();
#endif
};

class Event {
public:
  EventIndex event_id;
private:
  String event_name;
  HandlerFunc action;
  TriggerType type;
  TriggerList triggers;
  jet_time_t interval;
public:
#ifdef JET_EVT_MEASURE_TIMING_FUNC
  jet_time_t start;
  jet_time_t end;
  jet_time_t delta;
  jet_time_t peak_delta;
#endif
  Event(EventIndex id, HandlerFunc func, TriggerType type_in, jet_time_t interval_in = (uint32_t)0) :
    event_id(id), event_name(id), action(func), type(type_in), interval(interval_in) {}
  ~Event() {
    for (unsigned int trig_idx = 0; trig_idx < triggers.size(); trig_idx++) {
      delete(triggers.get(trig_idx));
    }
  }
  bool change_parameters(HandlerFunc func, TriggerType type) {
    if (func == nullptr) {
      return false;
    }
    this->action = func;
    this->type = type;
    // TODO: allow updating interval (requires DeltaClock integration)
    //this->interval = interval;
    reset_triggers();
    return true;
  }
  size_t get_trigger_count() {
    return triggers.size();
  }
  Trigger* get_trigger(int raw_index) {
    return triggers.get(raw_index);
  }
  Trigger* find_trigger(EventIndex id) {
    for (unsigned int trig_idx = 0; trig_idx < triggers.size(); trig_idx++) {
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
    for (unsigned int trig_idx = 0; trig_idx < triggers.size(); trig_idx++) {
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
      for (unsigned int trig_idx = 0; trig_idx < triggers.size(); trig_idx++) {
        if (triggers.get(trig_idx)->data_ready) {
          return true;
        }
      }
      return false;
    case TRIGGER_ON_ALL:
      for (unsigned int trig_idx = 0; trig_idx < triggers.size(); trig_idx++) {
        if (!triggers.get(trig_idx)->data_ready) {
          return false;
        }
      }
      return true;
    }
    return false;
  }
  bool take_action(Datum& result) {
#ifdef JET_EVT_MEASURE_TIMING_FUNC
    start = JET_EVT_MEASURE_TIMING_FUNC();
#endif
    bool data_generated = action(triggers, result);
#ifdef JET_EVT_MEASURE_TIMING_FUNC
    end = JET_EVT_MEASURE_TIMING_FUNC();
#endif
    reset_triggers();
#ifdef JET_EVT_MEASURE_TIMING_FUNC
    delta = end - start;
    if (delta > peak_delta) { peak_delta = delta; }
#endif
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
  void debug_string(String& out) {
    out += F("  ");
    out += this->event_name;
    out += ", type:";
    switch (type) {
      case TRIGGER_MANUAL:
        out += "TRIGGER_MANUAL";
        break;
      case TRIGGER_ON_ALL:
        out += "TRIGGER_ON_ALL";
        break;
      case TRIGGER_ON_ANY:
        out += "TRIGGER_ON_ANY";
        break;
#ifdef JET_EVT_HUB_TEMPORAL
      case TRIGGER_TEMPORAL:
        out += "TRIGGER_TEMPORAL";
        break;
#endif
      default:
        out += "unknown!";
        break;
    }
#ifdef JET_EVT_HUB_TEMPORAL
    if (type == TRIGGER_TEMPORAL) {
      out += " interval:";
      out += String(this->interval.to_uint32_t());
    }
#ifdef JET_EVT_MEASURE_TIMING_FUNC
    out += " timing stats:start=";
    out += String(this->start);
    out += ",end=";
    out += String(this->end);
    out += ",delta=";
    out += String(this->delta);
    out += ",peak_delta=";
    out += String(this->peak_delta);
#endif
#endif
    out += "\n";
    for (unsigned int idx = 0; idx < this->triggers.size(); idx++) {
      triggers.get(idx)->debug_string(out);
    }
  }
#ifdef JET_TEST
  friend bool HubTest();
#endif
};

class Hub {
private:
  EventList m_event_list;
  Event* find_event(EventIndex id) {
    for (unsigned int idx = 0; idx < m_event_list.size(); idx++) {
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
      context->hub->deliver(context->event->event_id, datum);
    }
  }
#endif
public:
  Hub() {}
  ~Hub() {
    for (unsigned int evt_idx = 0; evt_idx < m_event_list.size(); evt_idx++) {
      delete(m_event_list.get(evt_idx));
    }
#ifdef JET_EVT_HUB_TEMPORAL
    for (unsigned int idx = 0; idx < m_clock_entries.size(); idx++) {
      free(m_clock_entries.get(idx));
    }
    for (unsigned int idx = 0; idx < m_clock_handler_contexts.size(); idx++) {
      free(m_clock_handler_contexts.get(idx));
    }
#endif
  }
#ifdef JET_EVT_HUB_TEMPORAL
  void update(jet_time_t new_now) {
    clock.update(new_now);
  }
#endif
  bool add_event(EventIndex id, HandlerFunc func, TriggerType type, jet_time_t interval = (uint32_t)0) {
    Event* event = find_event(id);
    if (event != nullptr) {
      event->change_parameters(func, type);
      // TODO: apply new interval to DeltaClock entry
      return false;
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
    for (unsigned int event_idx = 0; event_idx < m_event_list.size(); event_idx++) {
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
  bool is_dag() {
    // Topological sort for cycle detection
    // DFS
    // https://en.wikipedia.org/wiki/Topological_sorting
    // Handlers are Nodes
    // Triggers are Edges
    // My DFS
    class VisitManager {
    public:
      static bool HasCycle(jet::PointerList<Event>& SeenNodes, Hub* hub, Event* CurrentNode) {
        if (SeenNodes.find_first(CurrentNode) != -1) {
          return true;
        }
        SeenNodes.append(CurrentNode);
        for (unsigned int trig_idx = 0; trig_idx < CurrentNode->get_trigger_count(); trig_idx++) {
          Trigger* trigger = CurrentNode->get_trigger((signed)trig_idx);
          Event* handler = hub->find_event(trigger->event_id);
          // Since events can be delivered externally, they can be missing from the graph as a node.
          if (handler != nullptr && HasCycle(SeenNodes, hub, handler)) {
            return true;
          }
        }
        SeenNodes.remove_first(CurrentNode);
        return false;
      }
    };
    // For each node, do a full cycle-sensitive depth-first graph traversal
    // There are faster or more theoretically sound ways to do this, but this
    // is accurate and simple. It even handles disconnected graphs.
    for (unsigned int handler_idx = 0; handler_idx < m_event_list.size(); handler_idx++) {
      jet::PointerList<Event> seen_nodes;
      Event* handler = m_event_list.get(handler_idx);
      if (VisitManager::HasCycle(seen_nodes, this, handler)) {
        return false;
      }
    }
    return true;
  }
  void debug_string(String& out) {
    out += "jet::evt::Hub(@0x";
    out += String((uintptr_t)this, HEX);
    out += ")\n";
    for (unsigned int idx = 0; idx < m_event_list.size(); idx++) {
      m_event_list.get(idx)->debug_string(out);
    }
#ifdef JET_EVT_HUB_TEMPORAL
    clock.debug_string(out);
    out += "\n";
#endif
    out += (is_dag() ? "Is DAG." : "Not DAG.");
    out += "\n";
  }
#ifdef JET_TEST
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
    jet_dbgprint(F("hub trigger: constructor/destructor"));
    Trigger* tr = new Trigger("hello");
    jet_assert(tr != nullptr);
    jet_assert(tr->event_id == "hello");
    jet_assert(tr->event_name.equals("hello"));
    jet_assert(tr->data.in16 == 0);
    jet_assert(tr->data_ready == false);
    delete(tr);
  }

  if (success) {
    jet_dbgprint(F("hub trigger: deliver/reset"));
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
    jet_dbgprint(F("hub event: constructor/destructor"));
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
    jet_dbgprint(F("hub event: add/find trigger"));
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
    jet_assert(ev->change_parameters(&TestHandlerProducer, TRIGGER_ON_ALL));
    jet_assert(ev->action == &TestHandlerProducer);
    jet_assert(ev->type == TRIGGER_ON_ALL);
    delete(ev);
  }

  if (success) {
    Event* ev = new Event("Halooo", &TestHandler, TRIGGER_MANUAL);
    jet_dbgprint(F("hub event: reset"));
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
      jet_dbgprint(F("hub event: take_action"));
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
      jet_dbgprint(F("hub event: deliver"));
      jet_assert(ev->triggers.get(0)->data_ready == false);
      jet_assert(ev->triggers.get(0)->data.uin16 == 0xdead);
      jet_assert(!ev->deliver_trigger(F("DoesNotExist"), datum, datum));
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
    jet_dbgprint(F("hub event: trigger firing and conditions"));
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
    jet_dbgprint(F("hub: constructor/destructor"));
    Hub* hub = new Hub();
    jet_assert(hub != nullptr);
    delete(hub);
  }

  if(success) {
    Hub* hub = new Hub();
    if (success) {
      jet_dbgprint(F("hub: normal Event insertion"));
      TestHandlerCounter = 0;
      jet_assert(hub->add_event("Event1", TestHandler, TRIGGER_ON_ANY));
      jet_assert(hub->m_event_list.size() == 1);
      jet_assert(hub->m_event_list.get(0)->event_id == "Event1");
      jet_assert(hub->m_event_list.get(0)->type == TRIGGER_ON_ANY);
      jet_assert(!hub->add_event("Event1", TestHandlerProducer, TRIGGER_ON_ALL));
      jet_assert(hub->m_event_list.size() == 1);
      jet_assert(hub->m_event_list.get(0)->event_id == "Event1");
      jet_assert(hub->m_event_list.get(0)->action == &TestHandlerProducer);
      jet_assert(hub->m_event_list.get(0)->type == TRIGGER_ON_ALL);
    }
    if (success) {
      jet_dbgprint(F("hub: trigger insertion"));
      jet_assert(hub->add_event_trigger("Event1", "Event1Trigger1"));
      jet_assert(hub->find_event("Event1") != nullptr);
      jet_assert(hub->find_event("Event1")->find_trigger("Event1Trigger1") != nullptr);
    }
    if (success) {
      jet_dbgprint(F("hub: temporal insertion"));
      jet_assert(hub->add_event("Event2", TestHandler, TRIGGER_TEMPORAL, 1000UL));
      jet_assert(hub->find_event("Event2") != nullptr);
    }
    if (success) {
      TestHandlerCounter = 0;
      jet_dbgprint(F("hub: update"));
      hub->update(0);
      hub->update(1000);
      jet_assert(TestHandlerCounter == 1);
      hub->update(2000);
      jet_assert(TestHandlerCounter == 2);
    }
    if (!success) {
      String str;
      jet_dbgprint(F("TestHandlerCounter: %d"), TestHandlerCounter);
      hub->debug_string(str);
      jet_dbgprint(F("%s"), str.c_str());
    }
    delete(hub);
  }

  if (success) {
    TestHandlerCounter = 0;
    Hub* hub = new Hub();
    Datum datum = { 0 };
    hub->update(0);
    jet_dbgprint(F("hub: delivery"));
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
    jet_dbgprint(F("hub: temporaldelivery"));
    jet_assert(hub->add_event("Timer1", &TestHandlerProducer, TRIGGER_TEMPORAL, 1000UL));
    jet_assert(hub->add_event_trigger("OnAny", "Timer1"));
    if (success) {
      hub->update((jet_time_t)1000);
      // Timer1 TestHandlerProducer fires and
      // triggers OnAny TestHandlerProducer
      jet_assert(TestHandlerCounter == 8);
    }
    jet_dbgprint("hub: debug_string");
    String str;
    hub->debug_string(str);
    jet_dbgprint("%s", str.c_str());
    jet_assert(str.length() != 0);
    jet_dbgprint("hub: debug_string (end)");
    delete(hub);
  }

  return success;
}

#endif // JET_TEST

} // namespace evt

} // namespace jet
