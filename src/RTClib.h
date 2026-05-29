#ifndef __RTCLIB_H__
#define __RTCLIB_H__

#include <stdint.h>
#include <time.h>
#include <Arduino.h>
#include <Wire.h>

namespace __rtclib_details {
  template <typename RTC>
  class RAMRef {
    RTC *_rtc;
    uint8_t _index;

  public:
    RAMRef(RTC *rtc, uint8_t index) : _rtc {rtc}, _index {index} {}

    operator uint8_t() { return _rtc->readRAM(_index); }
    operator int() { return _rtc->readRAM(_index); }

    RAMRef &operator=(uint8_t val) {
      _rtc->writeRAM(_index, val);
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
      uint8_t ret = *this;
      ++*this;
      return ret;
    }

    // Postfix decrement
    uint8_t operator--(int) {
      uint8_t ret = *this;
      --*this;
      return ret;
    }
  };

  template <typename RTC>
  class RAMPtr {
    RTC *_rtc;
    uint8_t _index;

  public:
    RAMPtr(RTC *rtc, uint8_t index) : _rtc {rtc}, _index {index} {}

    explicit operator uint8_t() const { return _index; }
    explicit operator int() const { return _index; }

    bool operator==(const RAMPtr &other) const { return _index == other._index; }
    bool operator!=(const RAMPtr &other) const { return _index != other._index; }
    bool operator<(const RAMPtr &other) const { return _index < other._index; }
    bool operator<=(const RAMPtr &other) const { return _index <= other._index; }
    bool operator>(const RAMPtr &other) const { return _index > other._index; }
    bool operator>=(const RAMPtr &other) const { return _index >= other._index; }

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
    RAMPtr operator+(int8_t off) const { return RAMPtr {_rtc, _index + off}; }
    friend RAMPtr operator+(int8_t off, const RAMPtr &other) { return other + off; }
    RAMPtr operator-(int8_t off) const { return RAMPtr {_rtc, _index - off}; }
    int8_t operator-(const RAMPtr &other) const {
      return static_cast<int8_t>(_index) - static_cast<int8_t>(other._index);
    }

    RAMRef<RTC> operator*() { return RAMRef<RTC> {_rtc, _index}; }

    RAMPtr operator++(int) { return RAMPtr {_rtc, _index++}; }
    RAMPtr operator--(int) { return RAMPtr {_rtc, _index--}; }
    RAMPtr &operator++() {
      ++_index;
      return *this;
    }
    RAMPtr &operator--() {
      --_index;
      return *this;
    }
  };
} // namespace __rtclib_details

class DS1302 {
  using RAMRef = __rtclib_details::RAMRef<DS1302>;
  using RAMPtr = __rtclib_details::RAMPtr<DS1302>;

  uint8_t _ce;
  uint8_t _sck;
  uint8_t _io;

  uint8_t _read();
  void _write(uint8_t val);

public:
  enum RegAddr : uint8_t {
    REG_W_SEC = 0x80,
    REG_R_SEC = 0x81,
    REG_W_MIN = 0x82,
    REG_R_MIN = 0x83,
    REG_W_HR = 0x84,
    REG_R_HR = 0x85,
    REG_W_DATE = 0x86,
    REG_R_DATE = 0x87,
    REG_W_MON = 0x88,
    REG_R_MON = 0x89,
    REG_W_DOW = 0x8a,
    REG_R_DOW = 0x8b,
    REG_W_YEAR = 0x8c,
    REG_R_YEAR = 0x8d,
    REG_W_WP = 0x8e,
    REG_R_WP = 0x8f,
    REG_W_TC = 0x90,
    REG_R_TC = 0x91,
    REG_W_CLKBURST = 0xbe,
    REG_R_CLKBURST = 0xbf,
    REG_W_RAM = 0xc0,
    REG_R_RAM = 0xc1,
    REG_W_RAMBURST = 0xfe,
    REG_R_RAMBURST = 0xff,
  };

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
  DS1302(const DS1302 &) = delete;
  DS1302 &operator=(const DS1302 &) = delete;

  bool setup();

  uint8_t readReg(RegAddr addr);
  void writeReg(RegAddr addr, uint8_t val);

  uint8_t readRAM(uint8_t index);
  void writeRAM(uint8_t index, uint8_t val);

  void getTime(tm *timeptr);
  void setTime(const tm *timeptr);

  bool isRunning();
  void setRunning(bool running);

  TrickleChargerMode getTrickleCharger();
  void setTrickleCharger(TrickleChargerMode value);

  RAMPtr begin() { return RAMPtr {this, 0}; }
  RAMPtr end() { return RAMPtr {this, RAM_SIZE}; }
  RAMRef operator[](uint8_t index) { return RAMRef {this, index}; }
};

class DS1307 {
  using RAMRef = __rtclib_details::RAMRef<DS1307>;
  using RAMPtr = __rtclib_details::RAMPtr<DS1307>;

  TwoWire &_wire;

public:
  enum RegAddr : uint8_t {
    REG_SEC = 0x00,
    REG_MIN = 0x01,
    REG_HR = 0x02,
    REG_DOW = 0x03,
    REG_DATE = 0x04,
    REG_MON = 0x05,
    REG_YEAR = 0x06,
    REG_CTRL = 0x07,
    REG_RAM = 0x08,
  };

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
  DS1307(const DS1307 &) = delete;
  DS1307 &operator=(const DS1307 &) = delete;

  bool setup();

  uint8_t readReg(RegAddr addr);
  void writeReg(RegAddr addr, uint8_t val);

  uint8_t readRAM(uint8_t index);
  void writeRAM(uint8_t index, uint8_t val);

  void getTime(tm *timeptr);
  void setTime(const tm *timeptr);

  bool isRunning();
  void setRunning(bool running);

  SqWaveFreq getSQWOut();
  void setSQWOut(SqWaveFreq value);

  RAMPtr begin() { return RAMPtr {this, 0}; }
  RAMPtr end() { return RAMPtr {this, RAM_SIZE}; }
  RAMRef operator[](uint8_t index) { return RAMRef {this, index}; }
};

class DS3231 {
  TwoWire &_wire;

public:
  enum RegAddr : uint8_t {
    REG_SEC = 0x00,
    REG_MIN = 0x01,
    REG_HR = 0x02,
    REG_DOW = 0x03,
    REG_DATE = 0x04,
    REG_MON = 0x05,
    REG_YEAR = 0x06,
    REG_AL1_SEC = 0x07,
    REG_AL1_MIN = 0x08,
    REG_AL1_HR = 0x09,
    REG_AL1_DATE = 0x0a,
    REG_AL2_MIN = 0x0b,
    REG_AL2_HR = 0x0c,
    REG_AL2_DATE = 0x0d,
    REG_CTRL = 0x0e,
    REG_STATUS = 0x0f,
    REG_AGING = 0x10,
    REG_TEMP_MSB = 0x11,
    REG_TEMP_LSB = 0x12,
  };

  enum IntSqwFreq : uint8_t {
    INT_EN = 0x04,
    SQW_1HZ = 0x00,
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
  DS3231(const DS3231 &) = delete;
  DS3231 &operator=(const DS3231 &) = delete;

  bool setup();

  uint8_t readReg(RegAddr addr);
  void writeReg(RegAddr addr, uint8_t val);

  void getTime(tm *timeptr);
  void setTime(const tm *timeptr);

  bool isRunning();
  void setRunning(bool running);

  bool getBBSQW();
  void setBBSQW(bool bbsqw);

  IntSqwFreq getIntSqw();
  void setIntSqw(IntSqwFreq freq);

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
  using RAMRef = __rtclib_details::RAMRef<RX8025T>;
  using RAMPtr = __rtclib_details::RAMPtr<RX8025T>;

  friend class __rtclib_details::RAMRef<RX8025T>;
  uint8_t readRAM(uint8_t) { return getRAM(); }
  void writeRAM(uint8_t, uint8_t val) { setRAM(val); }

  TwoWire &_wire;

public:
  enum RegAddr : uint8_t {
    REG_SEC = 0x00,
    REG_MIN = 0x01,
    REG_HOUR = 0x02,
    REG_WEEK = 0x03,
    REG_DAT = 0x04,
    REG_MONTH = 0x05,
    REG_YEAR = 0x06,
    REG_RAM = 0x07,
    REG_AL_MIN = 0x08,
    REG_AL_HOUR = 0x09,
    REG_AL_WK_D = 0x0a,
    REG_TIM0 = 0x0b,
    REG_TIM1 = 0x0c,
    REG_EXT = 0x0d,
    REG_FLAG = 0x0e,
    REG_CTRL = 0x0f,
  };

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
  static constexpr uint8_t RAM_SIZE = 1;

  explicit RX8025T(TwoWire &wire = Wire);
  RX8025T(const RX8025T &) = delete;
  RX8025T &operator=(const RX8025T &) = delete;

  bool setup();

  uint8_t readReg(RegAddr addr);
  void writeReg(RegAddr addr, uint8_t val);

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

  RAMPtr begin() { return RAMPtr {this, 0}; }
  RAMPtr end() { return RAMPtr {this, RAM_SIZE}; }
  RAMRef operator[](uint8_t index) { return RAMRef {this, index}; }
};

class PCF8563 {
  TwoWire &_wire;

public:
  enum RegAddr : uint8_t {
    REG_CTRL_1 = 0x00,
    REG_CTRL_2 = 0x01,
    REG_VL_SEC = 0x02,
    REG_MIN = 0x03,
    REG_HOUR = 0x04,
    REG_DAY = 0x05,
    REG_WEEK = 0x06,
    REG_CEN_MON = 0x07,
    REG_YEAR = 0x08,
    REG_AL_MIN = 0x09,
    REG_AL_HOUR = 0x0a,
    REG_AL_DAY = 0x0b,
    REG_AL_WEEK = 0x0c,
    REG_CLKOUT = 0x0d,
    REG_TIM_CTRL = 0x0e,
    REG_TIM = 0x0f,
  };

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
  PCF8563(const PCF8563 &) = delete;
  PCF8563 &operator=(const PCF8563 &) = delete;

  bool setup();

  uint8_t readReg(RegAddr addr);
  void writeReg(RegAddr addr, uint8_t val);

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
