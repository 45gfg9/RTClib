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
    RAMRef(T *thisPtr, uint8_t index) : _thisPtr {thisPtr}, _index {index} {}

    operator uint8_t() { return _thisPtr->readRAM(_index); }

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
    RAMPtr(T *thisPtr, uint8_t index) : _thisPtr {thisPtr}, _index {index} {}

    explicit operator uint8_t() const { return _index; }
    RAMPtr &operator=(uint8_t index) {
      _index = index;
      return *this;
    }
    RAMPtr &operator+=(int8_t off) {
      _index += off;
      return *this;
    }
    RAMPtr &operator-=(int8_t off) {
      _index -= off;
      return *this;
    }
    RAMPtr operator+(int8_t off) const { return RAMPtr(_thisPtr, _index + off); }
    friend RAMPtr operator+(int8_t off, const RAMPtr &other) { return other + off; }
    RAMPtr operator-(int8_t off) const { return RAMPtr(_thisPtr, _index - off); }
    int8_t operator-(const RAMPtr &other) const { return (int8_t)_index - (int8_t)other._index; }

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
  using RAMRef = __rtclib_details::RAMRef<DS1302>;
  using RAMPtr = __rtclib_details::RAMPtr<DS1302>;

  uint8_t _ce;
  uint8_t _sck;
  uint8_t _io;

  uint8_t _read();
  void _write(uint8_t val);

public:
  enum TrickleChargerMode : uint8_t {
    TC_OFF = 0x5c,  // off
    TC_1D2K = 0xa5, // 1 diode, 2K ohm
    TC_1D4K = 0xa6, // 1 diode, 4K ohm
    TC_1D8K = 0xa7, // 1 diode, 8K ohm
    TC_2D2K = 0xa9, // 2 diodes, 2K ohm
    TC_2D4K = 0xaa, // 2 diodes, 4K ohm
    TC_2D8K = 0xab, // 2 diodes, 8K ohm
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

  TrickleChargerMode getTrickleCharger();
  void setTrickleCharger(TrickleChargerMode value);

  RAMPtr begin() { return RAMPtr(this, 0); }
  RAMPtr end() { return RAMPtr(this, RAM_SIZE); }
  RAMRef operator[](int index) { return RAMRef(this, index); }
};

class DS1307 {
  using RAMRef = __rtclib_details::RAMRef<DS1307>;
  using RAMPtr = __rtclib_details::RAMPtr<DS1307>;

  TwoWire &_wire;

public:
  enum SqWaveFreq : uint8_t {
    SO_LOW = 0x00,   // keep sqw pin low
    SO_1HZ = 0x10,   // 1 Hz square wave
    SO_4KHZ = 0x11,  // 4.096 kHz square wave
    SO_8KHZ = 0x12,  // 8.192 kHz square wave
    SO_32KHZ = 0x13, // 32.768 kHz square wave
    SO_HIGH = 0x80,  // keep sqw pin high
  };

  static constexpr uint8_t ADDRESS = 0x68;
  static constexpr uint8_t RAM_SIZE = 56;

  explicit DS1307(TwoWire &wire = Wire);

  bool setup();

  uint8_t readReg(uint8_t addr);
  void writeReg(uint8_t addr, uint8_t val);

  uint8_t readRAM(uint8_t index);
  void writeRAM(uint8_t index, uint8_t val);

  void getTime(tm *timeptr);
  void setTime(const tm *timeptr);

  bool isRunning();
  void setRunning(bool running);

  SqWaveFreq getSQWOut();
  void setSQWOut(SqWaveFreq value);

  RAMPtr begin() { return RAMPtr(this, 0); }
  RAMPtr end() { return RAMPtr(this, RAM_SIZE); }
  RAMRef operator[](int index) { return RAMRef(this, index); }
};

class DS3231 {
  TwoWire &_wire;

public:
  enum SqWaveFreq : uint8_t {
    SQW_1HZ = 0x0,
    SQW_1024HZ = 0x08,
    SQW_4096HZ = 0x10,
    SQW_8192HZ = 0x18,
  };

  enum Alarm1Rate : uint8_t {
    AL1_EVERY_SECOND = 0x0f,
    AL1_MATCH_SECONDS = 0x0e,
    AL1_MATCH_MINUTES = 0x0c,
    AL1_MATCH_HOURS = 0x08,
    AL1_MATCH_DATE = 0x00,
    AL1_MATCH_DAY = 0x10,
    AL1_INVALID = 0xff,
  };

  enum Alarm2Rate : uint8_t {
    AL2_EVERY_MINUTE = 0x07,
    AL2_MATCH_MINUTES = 0x06,
    AL2_MATCH_HOURS = 0x04,
    AL2_MATCH_DATE = 0x00,
    AL2_MATCH_DAY = 0x08,
    AL2_INVALID = 0xff,
  };

  static constexpr uint8_t ADDRESS = 0x68;

  explicit DS3231(TwoWire &wire = Wire);

  bool setup();

  uint8_t readReg(uint8_t addr);
  void writeReg(uint8_t addr, uint8_t val);

  void getTime(tm *timeptr);
  void setTime(const tm *timeptr);

  bool isRunning();
  void setRunning(bool running);

  bool getINTCN();
  void setINTCN(bool intcn);

  bool getBBSQW();
  void setBBSQW(bool bbsqw);

  SqWaveFreq getSQWFreq();
  void setSQWFreq(SqWaveFreq freq);

  bool isIntrEnabled();
  void setIntrEnabled(bool enabled);

  Alarm1Rate getAL1(tm *timeptr);
  void setAL1(Alarm1Rate rate, const tm *timeptr);
  bool isAL1IntrEnabled();
  void setAL1IntrEnabled(bool enabled);
  bool getAL1IntrFlag();
  void clearAL1IntrFlag();

  Alarm2Rate getAL2(tm *timeptr);
  void setAL2(Alarm2Rate rate, const tm *timeptr);
  bool isAL2IntrEnabled();
  void setAL2IntrEnabled(bool enabled);
  bool getAL2IntrFlag();
  void clearAL2IntrFlag();

  int8_t getAgingOffset();
  void setAgingOffset(int8_t offset);

  float getTemperature();
};

// RX8025T: only basic timekeeping functions are stable
// other functions are subject to change
class RX8025T {
  TwoWire &_wire;

public:
  enum TempCompIntv : uint8_t {
    TC_0S5 = 0x00,
    TC_2S = 0x40,
    TC_10S = 0x80,
    TC_30S = 0xc0,
  };

  enum AlarmDay : uint8_t {
    AL_SUN = 0x81,
    AL_MON = 0x82,
    AL_TUE = 0x84,
    AL_WED = 0x88,
    AL_THU = 0x90,
    AL_FRI = 0xa0,
    AL_SAT = 0xc0,
    AL_EVERY_DAY = 0xff,
  };

  enum TimerFreq : uint8_t {
    TF_4096HZ = 0x00,
    TF_64HZ = 0x01,
    TF_1HZ = 0x02,
    TF_MINUTE = 0x03,
    TF_OFF = 0xff,
  };

  enum FOUTFreq : uint8_t {
    FOUT_32768HZ = 0x00,
    FOUT_1024HZ = 0x04,
    FOUT_1HZ = 0x08,
  };

  static constexpr uint8_t ADDRESS = 0x32;

  explicit RX8025T(TwoWire &wire = Wire);

  bool setup();

  uint8_t readReg(uint8_t addr);
  void writeReg(uint8_t addr, uint8_t val);

  void getTime(tm *timeptr);
  void setTime(const tm *timeptr);

  bool isRunning();
  void setRunning(bool running);

  TempCompIntv getTempCompInterval();
  void setTempCompIntv(TempCompIntv interval);

  uint8_t getRAM();
  void setRAM(uint8_t val);

  uint16_t getTimer();
  void setTimer(uint16_t val);
  TimerFreq getTimerFreq();
  void setTimerFreq(TimerFreq freq);
  bool isTimerIntrEnabled();
  void setTimerIntrEnabled(bool enabled);
  bool getTimerFlag();
  void clearTimerFlag();

  FOUTFreq getFOUT();
  void setFOUT(FOUTFreq freq);

  bool getVLF();
  void clearVLF();
  bool getVDET();
  void clearVDET();
  bool getUpdateFlag();
  void clearUpdateFlag();
  bool getUSEL();
  void setUSEL(bool usel);

  // alarm api is subject to change
  void getAlarm(tm *timeptr);
  void setAlarm(const tm *timeptr);
  bool isAlarmIntrEnabled();
  void setAlarmIntrEnabled(bool enabled);
  bool getAlarmFlag();
  void clearAlarmFlag();
};

class PCF8563 {
  TwoWire &_wire;

public:
  enum CLKFreq : uint8_t {
    CLKOUT_OFF = 0x00,
    CLKOUT_32768HZ = 0x80,
    CLKOUT_1024HZ = 0x81,
    CLKOUT_32HZ = 0x82,
    CLKOUT_1HZ = 0x83,
  };

  enum TimerFreq : uint8_t {
    TF_OFF = 0x00,
    TF_4096HZ = 0x80,
    TF_64HZ = 0x81,
    TF_1HZ = 0x82,
    TF_MINUTE = 0x83,
  };

  static constexpr uint8_t ADDRESS = 0x51;

  explicit PCF8563(TwoWire &wire = Wire);

  bool setup();

  uint8_t readReg(uint8_t addr);
  void writeReg(uint8_t addr, uint8_t val);

  void getTime(tm *timeptr);
  void setTime(const tm *timeptr);

  bool isRunning();
  void setRunning(bool running);

  CLKFreq getCLKOut();
  void setCLKOut(CLKFreq freq);

  uint8_t getTimer();
  void setTimer(uint8_t val);
  TimerFreq getTimerFreq();
  void setTimerFreq(TimerFreq freq);
  bool isTimerIntrEnabled();
  void setTimerIntrEnabled(bool enabled);
  bool getTimerFlag();
  void clearTimerFlag();
  bool isTimerPulseMode();
  void setTimerPulseMode(bool pulse_mode);

  void getAlarm(tm *timeptr);
  void setAlarm(const tm *timeptr);
  bool isAlarmIntrEnabled();
  void setAlarmIntrEnabled(bool enabled);
  bool getAlarmFlag();
  void clearAlarmFlag();
};

#endif
