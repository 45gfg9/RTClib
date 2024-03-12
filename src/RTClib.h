#ifndef __RTCLIB_H__
#define __RTCLIB_H__

#include <time.h>
#include <Arduino.h>
#include <Wire.h>

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

  uint8_t _read();
  void _write(uint8_t val);

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

class DS1307 {
  using RAMRef = __rtclib_details::RAMRef<DS1307>;
  using RAMPtr = __rtclib_details::RAMPtr<DS1307>;

  TwoWire &_wire;

public:
  enum sqw_out_t : uint8_t {
    SO_LOW = 0x00,
    SO_1HZ = 0x10,
    SO_4KHZ = 0x11,
    SO_8KHZ = 0x12,
    SO_32KHZ = 0x13,
    SO_HIGH = 0x80,
  };

  static constexpr uint8_t ADDRESS = 0x68;
  static constexpr uint8_t RAM_SIZE = 56;

  DS1307(TwoWire &wire = Wire);

  bool setup();

  uint8_t readReg(uint8_t addr);
  void writeReg(uint8_t addr, uint8_t val);

  uint8_t readRAM(uint8_t index);
  void writeRAM(uint8_t index, uint8_t val);

  void getTime(tm *timeptr);
  void setTime(const tm *timeptr);

  bool isRunning();
  void setRunning(bool running);

  sqw_out_t getSQWOut();
  void setSQWOut(sqw_out_t value);

  RAMPtr begin() { return RAMPtr(this, 0); }
  RAMPtr end() { return RAMPtr(this, RAM_SIZE); }
  RAMRef operator[](int index) { return RAMRef(this, index); }
};

class DS3231 {
  TwoWire &_wire;

public:
  enum sqw_t : uint8_t {
    SQW_1HZ = 0x0,
    SQW_1024HZ = 0x08,
    SQW_4096HZ = 0x10,
    SQW_8192HZ = 0x18,
  };

  enum alarm_1_rate : uint8_t {
    AL1_EVERY_SECOND = 0x0f,
    AL1_MATCH_SECONDS = 0x0e,
    AL1_MATCH_MINUTES = 0x0c,
    AL1_MATCH_HOURS = 0x08,
    AL1_MATCH_DATE = 0x00,
    AL1_MATCH_DAY = 0x10,
    AL1_INVALID = 0xff,
  };

  enum alarm_2_rate : uint8_t {
    AL2_EVERY_MINUTE = 0x07,
    AL2_MATCH_MINUTES = 0x06,
    AL2_MATCH_HOURS = 0x04,
    AL2_MATCH_DATE = 0x00,
    AL2_MATCH_DAY = 0x08,
    AL2_INVALID = 0xff,
  };

  static constexpr uint8_t ADDRESS = 0x68;

  DS3231(TwoWire &wire = Wire);

  bool setup();

  uint8_t readReg(uint8_t addr);
  void writeReg(uint8_t addr, uint8_t val);

  void getTime(tm *timeptr);
  void setTime(const tm *timeptr);

  bool isRunning();
  void setRunning(bool running);

  sqw_t getSQWFreq();
  void setSQWFreq(sqw_t freq);

  bool isInterruptEnabled();
  void setInterruptEnabled(bool enabled);

  bool isAlarm1InterruptEnabled();
  void setAlarm1InterruptEnabled(bool enabled);

  bool isAlarm2InterruptEnabled();
  void setAlarm2InterruptEnabled(bool enabled);

  alarm_1_rate getAlarm1(tm *timeptr);
  void setAlarm1(alarm_1_rate rate, const tm *timeptr);

  alarm_2_rate getAlarm2(tm *timeptr);
  void setAlarm2(alarm_2_rate rate, const tm *timeptr);

  int8_t getAgingOffset();
  void setAgingOffset(int8_t offset);

  float getTemperature();
};

#endif
