#ifndef __RTCLIB_H__
#define __RTCLIB_H__

#include <time.h>
#include <Arduino.h>

class DS1302 {
  template <typename T>
  class GenericRAMRef {
    T *_thisPtr;
    uint8_t _index;

  public:
    GenericRAMRef(T *thisPtr, uint8_t index) : _thisPtr(thisPtr), _index(index) {}

    operator uint8_t() const { return _thisPtr->readRAM(_index); }

    GenericRAMRef &operator=(const GenericRAMRef &ref) { return *this = *ref; }
    GenericRAMRef &operator=(uint8_t val) {
      _thisPtr->writeRAM(_index, val);
      return *this;
    }
    GenericRAMRef &operator+=(uint8_t in) { return *this = *this + in; }
    GenericRAMRef &operator-=(uint8_t in) { return *this = *this - in; }
    GenericRAMRef &operator*=(uint8_t in) { return *this = *this * in; }
    GenericRAMRef &operator/=(uint8_t in) { return *this = *this / in; }
    GenericRAMRef &operator^=(uint8_t in) { return *this = *this ^ in; }
    GenericRAMRef &operator%=(uint8_t in) { return *this = *this % in; }
    GenericRAMRef &operator&=(uint8_t in) { return *this = *this & in; }
    GenericRAMRef &operator|=(uint8_t in) { return *this = *this | in; }
    GenericRAMRef &operator<<=(uint8_t in) { return *this = *this << in; }
    GenericRAMRef &operator>>=(uint8_t in) { return *this = *this >> in; }

    // Prefix increment
    GenericRAMRef &operator++() { return *this += 1; }
    // Prefix decrement
    GenericRAMRef &operator--() { return *this -= 1; }

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
  class GenericRAMPtr {
    T *_thisPtr;
    uint8_t _index;

  public:
    GenericRAMPtr(T *thisPtr, uint8_t index) : _thisPtr(thisPtr), _index(index) {}

    operator uint8_t() const { return _index; }
    GenericRAMPtr &operator=(uint8_t index) {
      _index = index;
      return *this;
    }

    GenericRAMRef<T> operator*() { return GenericRAMRef<T>(_thisPtr, _index); }

    GenericRAMPtr operator++(int) { return GenericRAMPtr(_thisPtr, _index++); }
    GenericRAMPtr operator--(int) { return GenericRAMPtr(_thisPtr, _index--); }
    GenericRAMPtr &operator++() {
      ++_index;
      return *this;
    }
    GenericRAMPtr &operator--() {
      --_index;
      return *this;
    }
  };

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

  using RAMRef = GenericRAMRef<DS1302>;
  using RAMPtr = GenericRAMPtr<DS1302>;

  uint8_t _ce;
  uint8_t _sck;
  uint8_t _io;

  uint8_t read();
  void write(uint8_t val);

  uint8_t readRAM(uint8_t index);
  void writeRAM(uint8_t index, uint8_t val);

public:
  static constexpr uint8_t RAM_SIZE = 31;

  void begin(uint8_t ce, uint8_t sck, uint8_t io);

  uint8_t readReg(uint8_t addr);
  void writeReg(uint8_t addr, uint8_t val);

  void getTime(tm *timeptr);
  void setTime(const tm *timeptr);

  bool isRunning();
  void setRunning(bool running);

  RAMPtr begin() { return RAMPtr(this, 0); }
  RAMPtr end() { return RAMPtr(this, RAM_SIZE); }
  RAMRef operator[](int index) { return RAMRef(this, index); }
};

#endif
