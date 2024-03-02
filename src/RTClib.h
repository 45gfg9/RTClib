#ifndef __RTCLIB_H__
#define __RTCLIB_H__

#include <time.h>
#include <Arduino.h>

namespace __rtclib_details {
  template <typename T>
  class RAMRef {
    T *_thisPtr;
    uint8_t _index;

  public:
    RAMRef(T *thisPtr, uint8_t index) : _thisPtr(thisPtr), _index(index) {}

    operator uint8_t() const { return _thisPtr->readRAM(_index); }

    RAMRef &operator=(const RAMRef &ref) { return *this = *ref; }
    RAMRef &operator=(uint8_t val) {
      _thisPtr->writeRAM(_index, val);
      return *this;
    }
    RAMRef &operator+=(uint8_t in) { return *this = *this + in; }
    RAMRef &operator-=(uint8_t in) { return *this = *this - in; }
    RAMRef &operator*=(uint8_t in) { return *this = *this * in; }
    RAMRef &operator/=(uint8_t in) { return *this = *this / in; }
    RAMRef &operator^=(uint8_t in) { return *this = *this ^ in; }
    RAMRef &operator%=(uint8_t in) { return *this = *this % in; }
    RAMRef &operator&=(uint8_t in) { return *this = *this & in; }
    RAMRef &operator|=(uint8_t in) { return *this = *this | in; }
    RAMRef &operator<<=(uint8_t in) { return *this = *this << in; }
    RAMRef &operator>>=(uint8_t in) { return *this = *this >> in; }

    // Prefix increment
    RAMRef &operator++() { return *this += 1; }
    // Prefix decrement
    RAMRef &operator--() { return *this -= 1; }

    // Postfix increment
    uint8_t operator++(int) {
      uint8_t ret = **this;
      ++*this;
      return ret;
    }

    // Postfix decrement
    uint8_t operator--(int) {
      uint8_t ret = **this;
      --*this;
      return ret;
    }
  };

  template <typename T>
  class RAMPtr {
    T *_thisPtr;
    uint8_t _index;

  public:
    RAMPtr(T *thisPtr, uint8_t index) : _thisPtr(thisPtr), _index(index) {}

    operator uint8_t() const { return _index; }
    RAMPtr &operator=(uint8_t index) {
      _index = index;
      return *this;
    }

    RAMRef<T> operator*() { return RAMRef<T>(_thisPtr, _index); }

    RAMPtr operator++(int) { return RAMPtr(_thisPtr, _index++); }
    RAMPtr operator--(int) { return RAMPtr(_thisPtr, _index--); }
    RAMPtr &operator++() {
      ++_index;
      return *this;
    }
    RAMPtr &operator--() {
      --_index;
      return *this;
    }
  };
}; // namespace __rtclib_details

class DS1302 {
  // RAII class for data transferring
  class TransferHelper {
    uint8_t _ce, _sck;

    static constexpr uint8_t ce_to_sck_setup = 4;
    static constexpr uint8_t ce_inactive_time = 4;

  public:
    TransferHelper(uint8_t ce, uint8_t sck) : _ce(ce), _sck(sck) {
      digitalWrite(_sck, LOW);
      digitalWrite(_ce, HIGH);
      delayMicroseconds(ce_to_sck_setup);
    }

    ~TransferHelper() {
      digitalWrite(_ce, LOW);
      delayMicroseconds(ce_inactive_time);
    }
  };

  using RAMRef = __rtclib_details::RAMRef<DS1302>;
  using RAMPtr = __rtclib_details::RAMPtr<DS1302>;

  uint8_t _ce;
  uint8_t _sck;
  uint8_t _io;

  uint8_t read();
  void write(uint8_t val);

public:
  enum trickle_charger_t : uint8_t {
    TC_OFF = 0x5c,
    TC_1D2K = 0xa5,
    TC_1D4K = 0xa6,
    TC_1D8K = 0xa7,
    TC_2D2K = 0xa9,
    TC_2D4K = 0xaa,
    TC_2D8K = 0xab,
  };

  static constexpr uint8_t RAM_SIZE = 31;

  DS1302(uint8_t ce, uint8_t sck, uint8_t io);

  bool setup();

  uint8_t readReg(uint8_t addr);
  void writeReg(uint8_t addr, uint8_t val);

  uint8_t readRAM(uint8_t index);
  void writeRAM(uint8_t index, uint8_t val);

  void getTime(tm *timeptr);
  void setTime(const tm *timeptr);

  bool isRunning();
  void setRunning(bool running);

  trickle_charger_t getTrickleCharger();
  void setTrickleCharger(trickle_charger_t value);

  RAMPtr begin() { return RAMPtr(this, 0); }
  RAMPtr end() { return RAMPtr(this, RAM_SIZE); }
  RAMRef operator[](int index) { return RAMRef(this, index); }
};

#endif
