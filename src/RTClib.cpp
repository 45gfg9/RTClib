#include "RTClib.h"

#ifndef likely
#ifdef __GNUC__
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#else
#define likely(x) (x)
#define unlikely(x) (x)
#endif
#endif

// RAII class for data transferring to/from DS1302
namespace {
  class TransferHelper {
    uint8_t _ce, _sck;

    static constexpr uint8_t CE_TO_SCK_SETUP = 4;
    static constexpr uint8_t CE_INACTIVE_TIME = 4;

  public:
    TransferHelper(uint8_t ce, uint8_t sck) : _ce {ce}, _sck {sck} {
      digitalWrite(_sck, LOW);
      digitalWrite(_ce, HIGH);
      delayMicroseconds(CE_TO_SCK_SETUP);
    }

    ~TransferHelper() {
      digitalWrite(_ce, LOW);
      delayMicroseconds(CE_INACTIVE_TIME);
    }
  };
} // namespace

static constexpr uint8_t bcd2bin(uint8_t val) {
  return val - 6 * (val >> 4);
}

static constexpr uint8_t bin2bcd(uint8_t val) {
  return val + 6 * (val / 10);
}

static void i2c_rtc_write(TwoWire &wire, uint8_t dev, uint8_t addr, uint8_t val) {
  wire.beginTransmission(dev);
  wire.write(addr);
  wire.write(val);
  wire.endTransmission();
}

static uint8_t i2c_rtc_read(TwoWire &wire, uint8_t dev, uint8_t addr) {
  wire.beginTransmission(dev);
  wire.write(addr);
  wire.endTransmission();

  wire.requestFrom(dev, uint8_t {1});
  return wire.read();
}

#define MASK_BOOL_REG_BITS(addr, bits, en)        \
  do {                                            \
    uint8_t __mask = (en) ? (bits) : 0;           \
    uint8_t __val = readReg(addr);                \
    if ((__val & (bits)) != __mask) {             \
      writeReg(addr, (__val & ~(bits)) | __mask); \
    }                                             \
  } while (0)

DS1302::DS1302(uint8_t ce, uint8_t sck, uint8_t io) : _ce {ce}, _sck {sck}, _io {io} {}

bool DS1302::setup() {
  pinMode(_ce, OUTPUT);
  pinMode(_sck, OUTPUT);
  writeReg(REG_W_WP, 0);
  setRunning(true);

  return true;
}

uint8_t DS1302::_read() {
  pinMode(_io, INPUT);

  // shiftIn() will not work
  if (false) {
    return shiftIn(_io, _sck, LSBFIRST);
  }

  uint8_t value = 0;
  for (uint8_t i = 8; i; --i) {
    uint8_t bit = digitalRead(_io);
    digitalWrite(_sck, HIGH);
    value = (value >> 1) | (bit ? 0x80 : 0); // LSB first
    delayMicroseconds(1);
    digitalWrite(_sck, LOW);
  }
  return value;
}

void DS1302::_write(uint8_t val) {
  pinMode(_io, OUTPUT);

  // shiftOut() will not work
  if (false) {
    return shiftOut(_io, _sck, LSBFIRST, val);
  }

  for (uint8_t i = 8; i; --i) {
    digitalWrite(_io, val & 1);
    digitalWrite(_sck, HIGH);
    val >>= 1;
    delayMicroseconds(1);
    digitalWrite(_sck, LOW);
  }
}

uint8_t DS1302::readReg(RegAddr addr) {
  TransferHelper _tr {_ce, _sck};

  _write(addr);
  return _read();
}

void DS1302::writeReg(RegAddr addr, uint8_t val) {
  TransferHelper _tr {_ce, _sck};

  _write(addr);
  _write(val);
}

void DS1302::getTime(tm *timeptr) {
  TransferHelper _tr {_ce, _sck};

  _write(REG_R_CLKBURST);
  timeptr->tm_sec = bcd2bin(_read() & 0x7f);
  timeptr->tm_min = bcd2bin(_read());
  timeptr->tm_hour = bcd2bin(_read());
  timeptr->tm_mday = bcd2bin(_read());
  timeptr->tm_mon = bcd2bin(_read()) - 1;
  timeptr->tm_wday = _read();
  timeptr->tm_year = bcd2bin(_read()) + 100;

  if (unlikely(timeptr->tm_wday == 7)) {
    // Sunday
    timeptr->tm_wday = 0;
  }
}

void DS1302::setTime(const tm *timeptr) {
  TransferHelper _tr {_ce, _sck};

  uint8_t wday = timeptr->tm_wday;
  if (unlikely(wday == 0)) {
    // Sunday
    wday = 7;
  }

  _write(REG_W_CLKBURST);
  _write(bin2bcd(timeptr->tm_sec));
  _write(bin2bcd(timeptr->tm_min));
  _write(bin2bcd(timeptr->tm_hour));
  _write(bin2bcd(timeptr->tm_mday));
  _write(bin2bcd(timeptr->tm_mon + 1));
  _write(wday);
  _write(bin2bcd(timeptr->tm_year - 100));
  _write(0);
}

bool DS1302::isRunning() {
  return (readReg(REG_R_SEC) & 0x80) == 0;
}

void DS1302::setRunning(bool running) {
  MASK_BOOL_REG_BITS(REG_R_SEC, 0x80, !running);
}

DS1302::TrickleChargerMode DS1302::getTrickleCharger() {
  uint8_t r = readReg(REG_R_TC);

  bool tcs = (r & 0xf0) == 0xa0;
  uint8_t ds = r & 0x0c;
  uint8_t rs = r & 0x03;

  // the register value might not always be valid
  if (tcs && rs != 0 && (ds == 0x04 || ds == 0x08)) {
    return TrickleChargerMode {r};
  }
  return TC_OFF;
}

void DS1302::setTrickleCharger(TrickleChargerMode mode) {
  writeReg(REG_W_TC, mode);
}

uint8_t DS1302::readRAM(uint8_t index) {
  if (index >= RAM_SIZE) {
    return 0;
  }

  TransferHelper _tr {_ce, _sck};

  _write(REG_R_RAM + (index << 1));
  return _read();
}

void DS1302::writeRAM(uint8_t index, uint8_t val) {
  if (index >= RAM_SIZE) {
    return;
  }

  TransferHelper _tr {_ce, _sck};

  _write(REG_W_RAM + (index << 1));
  _write(val);
}

DS1307::DS1307(TwoWire &wire) : _wire {wire} {}

bool DS1307::setup() {
  _wire.beginTransmission(ADDRESS);
  return _wire.endTransmission() == 0;
}

uint8_t DS1307::readReg(RegAddr addr) {
  return i2c_rtc_read(_wire, ADDRESS, addr);
}

void DS1307::writeReg(RegAddr addr, uint8_t val) {
  i2c_rtc_write(_wire, ADDRESS, addr, val);
}

uint8_t DS1307::readRAM(uint8_t index) {
  if (index >= RAM_SIZE) {
    return 0;
  }

  return i2c_rtc_read(_wire, ADDRESS, REG_RAM + index);
}

void DS1307::writeRAM(uint8_t index, uint8_t val) {
  if (index >= RAM_SIZE) {
    return;
  }

  i2c_rtc_write(_wire, ADDRESS, REG_RAM + index, val);
}

void DS1307::getTime(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_SEC);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t {7});
  timeptr->tm_sec = bcd2bin(_wire.read() & 0x7f);
  timeptr->tm_min = bcd2bin(_wire.read());
  timeptr->tm_hour = bcd2bin(_wire.read());
  timeptr->tm_wday = _wire.read();
  timeptr->tm_mday = bcd2bin(_wire.read());
  timeptr->tm_mon = bcd2bin(_wire.read()) - 1;
  timeptr->tm_year = bcd2bin(_wire.read()) + 100;

  if (unlikely(timeptr->tm_wday == 7)) {
    // Sunday
    timeptr->tm_wday = 0;
  }
}

void DS1307::setTime(const tm *timeptr) {
  uint8_t wday = timeptr->tm_wday;
  if (unlikely(wday == 0)) {
    // Sunday
    wday = 7;
  }

  const uint8_t write_buf[] {
      REG_SEC,
      bin2bcd(timeptr->tm_sec),
      bin2bcd(timeptr->tm_min),
      bin2bcd(timeptr->tm_hour),
      wday,
      bin2bcd(timeptr->tm_mday),
      bin2bcd(timeptr->tm_mon + 1),
      bin2bcd(timeptr->tm_year - 100),
  };

  _wire.beginTransmission(ADDRESS);
  _wire.write(write_buf, sizeof(write_buf));
  _wire.endTransmission();
}

bool DS1307::isRunning() {
  return (readReg(REG_SEC) & 0x80) == 0;
}

void DS1307::setRunning(bool running) {
  MASK_BOOL_REG_BITS(REG_SEC, 0x80, !running);
}

DS1307::SqWaveFreq DS1307::getSQWOut() {
  uint8_t r = readReg(REG_CTRL);
  return static_cast<SqWaveFreq>((r & 0x10) ? (r & 0x13) : (r & 0x80));
}

void DS1307::setSQWOut(SqWaveFreq value) {
  writeReg(REG_CTRL, value);
}

DS3231::DS3231(TwoWire &wire) : _wire {wire} {}

bool DS3231::setup() {
  _wire.beginTransmission(ADDRESS);
  return _wire.endTransmission() == 0;
}

uint8_t DS3231::readReg(RegAddr addr) {
  return i2c_rtc_read(_wire, ADDRESS, addr);
}

void DS3231::writeReg(RegAddr addr, uint8_t val) {
  i2c_rtc_write(_wire, ADDRESS, addr, val);
}

void DS3231::getTime(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_SEC);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t {7});
  timeptr->tm_sec = bcd2bin(_wire.read() & 0x7f);
  timeptr->tm_min = bcd2bin(_wire.read());
  timeptr->tm_hour = bcd2bin(_wire.read());
  timeptr->tm_wday = _wire.read();
  timeptr->tm_mday = bcd2bin(_wire.read());
  uint8_t cen_mon = _wire.read();
  timeptr->tm_mon = bcd2bin(cen_mon & 0x1f) - 1;
  timeptr->tm_year = bcd2bin(_wire.read()) + 100;

  if (unlikely(timeptr->tm_wday == 7)) {
    // Sunday
    timeptr->tm_wday = 0;
  }

  if (unlikely(cen_mon & 0x80)) {
    // century bit set
    timeptr->tm_year += 100;
  }
}

void DS3231::setTime(const tm *timeptr) {
  uint8_t wday = timeptr->tm_wday;
  if (unlikely(wday == 0)) {
    // Sunday
    wday = 7;
  }

  uint8_t year = timeptr->tm_year - 100;
  uint8_t cen_mon = bin2bcd(timeptr->tm_mon + 1);

  if (year >= 100) {
    cen_mon |= 0x80;
    year -= 100;
  }

  const uint8_t write_buf[] {
      REG_SEC,
      bin2bcd(timeptr->tm_sec),
      bin2bcd(timeptr->tm_min),
      bin2bcd(timeptr->tm_hour),
      wday,
      bin2bcd(timeptr->tm_mday),
      cen_mon,
      bin2bcd(year),
  };

  _wire.beginTransmission(ADDRESS);
  _wire.write(write_buf, sizeof(write_buf));
  _wire.endTransmission();
}

bool DS3231::isRunning() {
  return (readReg(REG_CTRL) & 0x80) == 0;
}

void DS3231::setRunning(bool running) {
  MASK_BOOL_REG_BITS(REG_CTRL, 0x80, !running);
}

bool DS3231::getBBSQW() {
  return (readReg(REG_CTRL) & 0x40) != 0;
}

void DS3231::setBBSQW(bool bbsqw) {
  MASK_BOOL_REG_BITS(REG_CTRL, 0x40, bbsqw);
}

DS3231::IntSqwFreq DS3231::getIntSqw() {
  uint8_t ctrl = readReg(REG_CTRL);
  return (ctrl & 0x04) ? INT_EN : static_cast<IntSqwFreq>(ctrl & 0x18);
}

void DS3231::setIntSqw(IntSqwFreq freq) {
  uint8_t ctrl = readReg(REG_CTRL);
  writeReg(REG_CTRL, (ctrl & ~0x1c) | freq);
}

bool DS3231::isAL1IntrEnabled() {
  return (readReg(REG_CTRL) & 0x01) != 0;
}

void DS3231::setAL1IntrEnabled(bool enabled) {
  MASK_BOOL_REG_BITS(REG_CTRL, 0x01, enabled);
}

bool DS3231::getAL1IntrFlag() {
  return (readReg(REG_STATUS) & 0x01) != 0;
}

void DS3231::clearAL1IntrFlag() {
  MASK_BOOL_REG_BITS(REG_STATUS, 0x01, 0);
}

bool DS3231::isAL2IntrEnabled() {
  return (readReg(REG_CTRL) & 0x02) != 0;
}

void DS3231::setAL2IntrEnabled(bool enabled) {
  MASK_BOOL_REG_BITS(REG_CTRL, 0x02, enabled);
}

bool DS3231::getAL2IntrFlag() {
  return (readReg(REG_STATUS) & 0x02) != 0;
}

void DS3231::clearAL2IntrFlag() {
  MASK_BOOL_REG_BITS(REG_STATUS, 0x02, 0);
}

DS3231::Alarm1Rate DS3231::getAL1(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_AL1_SEC);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t {4});
  uint8_t sec = _wire.read();
  uint8_t min = _wire.read();
  uint8_t hr = _wire.read();
  uint8_t date = _wire.read();

  bool dy_dt = date & 0x40;

  if (timeptr) {
    timeptr->tm_sec = bcd2bin(sec & 0x7f);
    timeptr->tm_min = bcd2bin(min & 0x7f);
    timeptr->tm_hour = bcd2bin(hr & 0x3f);

    if (dy_dt) {
      // DY/#DT bit set, match day of week
      timeptr->tm_wday = date & 0x07;
      if (unlikely(timeptr->tm_wday == 7)) {
        // Sunday
        timeptr->tm_wday = 0;
      }
    } else {
      // DY/#DT bit clear, match date
      timeptr->tm_mday = bcd2bin(date & 0x3f);
    }
  }

  bool a1m4 = date & 0x80;
  bool a1m3 = hr & 0x80;
  bool a1m2 = min & 0x80;
  bool a1m1 = sec & 0x80;

  if (a1m4 && a1m3 && a1m2 && a1m1) {
    return AL1_EVERY_SECOND;
  } else if (a1m4 && a1m3 && a1m2 && !a1m1) {
    return AL1_MATCH_SECONDS;
  } else if (a1m4 && a1m3 && !a1m2 && !a1m1) {
    return AL1_MATCH_MINUTES;
  } else if (a1m4 && !a1m3 && !a1m2 && !a1m1) {
    return AL1_MATCH_HOURS;
  } else if (!a1m4 && !a1m3 && !a1m2 && !a1m1 && !dy_dt) {
    return AL1_MATCH_DATE;
  } else if (!a1m4 && !a1m3 && !a1m2 && !a1m1 && dy_dt) {
    return AL1_MATCH_DAY;
  } else {
    return AL1_INVALID;
  }
}

void DS3231::setAL1(Alarm1Rate rate, const tm *timeptr) {
  uint8_t sec = bin2bcd(timeptr->tm_sec);
  uint8_t min = bin2bcd(timeptr->tm_min);
  uint8_t hr = bin2bcd(timeptr->tm_hour);
  uint8_t date = bin2bcd(timeptr->tm_mday);
  uint8_t wday = timeptr->tm_wday;
  if (unlikely(wday == 0)) {
    // Sunday
    wday = 7;
  }

  switch (rate) {
    case AL1_EVERY_SECOND:
      sec |= 0x80;
      [[fallthrough]];
    case AL1_MATCH_SECONDS:
      min |= 0x80;
      [[fallthrough]];
    case AL1_MATCH_MINUTES:
      hr |= 0x80;
      [[fallthrough]];
    case AL1_MATCH_HOURS:
      date |= 0x80;
      [[fallthrough]];
    default:
      break;
    case AL1_MATCH_DAY:
      date |= 0x40;
      break;
  }

  const uint8_t write_buf[] {
      REG_AL1_SEC,
      sec,
      min,
      hr,
      date,
  };

  _wire.beginTransmission(ADDRESS);
  _wire.write(write_buf, sizeof(write_buf));
  _wire.endTransmission();
}

DS3231::Alarm2Rate DS3231::getAL2(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_AL2_MIN);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t {3});
  uint8_t min = _wire.read();
  uint8_t hr = _wire.read();
  uint8_t date = _wire.read();

  bool dy_dt = date & 0x40;

  if (timeptr) {
    timeptr->tm_min = bcd2bin(min & 0x7f);
    timeptr->tm_hour = bcd2bin(hr & 0x3f);

    if (dy_dt) {
      // DY/#DT bit set, match day of week
      timeptr->tm_wday = date & 0x07;
      if (unlikely(timeptr->tm_wday == 7)) {
        // Sunday
        timeptr->tm_wday = 0;
      }
    } else {
      // DY/#DT bit clear, match date
      timeptr->tm_mday = bcd2bin(date & 0x3f);
    }
  }

  bool a2m4 = date & 0x80;
  bool a2m3 = hr & 0x80;
  bool a2m2 = min & 0x80;

  if (a2m4 && a2m3 && a2m2) {
    return AL2_EVERY_MINUTE;
  } else if (a2m4 && a2m3 && !a2m2) {
    return AL2_MATCH_MINUTES;
  } else if (a2m4 && !a2m3 && !a2m2) {
    return AL2_MATCH_HOURS;
  } else if (!a2m4 && !a2m3 && !a2m2 && !dy_dt) {
    return AL2_MATCH_DATE;
  } else if (!a2m4 && !a2m3 && !a2m2 && dy_dt) {
    return AL2_MATCH_DAY;
  } else {
    return AL2_INVALID;
  }
}

void DS3231::setAL2(Alarm2Rate rate, const tm *timeptr) {
  uint8_t min = bin2bcd(timeptr->tm_min);
  uint8_t hr = bin2bcd(timeptr->tm_hour);
  uint8_t date = bin2bcd(timeptr->tm_mday);
  uint8_t wday = timeptr->tm_wday;
  if (unlikely(wday == 0)) {
    // Sunday
    wday = 7;
  }

  switch (rate) {
    case AL2_EVERY_MINUTE:
      min |= 0x80;
      [[fallthrough]];
    case AL2_MATCH_MINUTES:
      hr |= 0x80;
      [[fallthrough]];
    case AL2_MATCH_HOURS:
      date |= 0x80;
      [[fallthrough]];
    default:
      break;
    case AL2_MATCH_DAY:
      // FIXME: wday not respected, same for AL1
      date |= 0x40;
      break;
  }

  const uint8_t write_buf[] {
      REG_AL2_MIN,
      min,
      hr,
      date,
  };

  _wire.beginTransmission(ADDRESS);
  _wire.write(write_buf, sizeof(write_buf));
  _wire.endTransmission();
}

int8_t DS3231::getAgingOffset() {
  return static_cast<int8_t>(readReg(REG_AGING));
}

void DS3231::setAgingOffset(int8_t offset) {
  writeReg(REG_AGING, static_cast<uint8_t>(offset));
}

float DS3231::getTemperature() {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_TEMP_MSB);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t {2});
  uint8_t msb = _wire.read();
  uint8_t lsb = _wire.read();

  int16_t temp = (msb << 8) | lsb;
  return temp / 256.0f;
}

RX8025T::RX8025T(TwoWire &wire) : _wire {wire} {}

bool RX8025T::setup() {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_FLAG);
  if (_wire.endTransmission() != 0) {
    return false;
  }

  _wire.requestFrom(ADDRESS, uint8_t {1});
  uint8_t flag = _wire.read();
  _wire.endTransmission();

  // check VLF
  if (flag & 0x02) {
    static constexpr uint8_t PROGMEM init_regs[] {
        REG_SEC,
        0x00, // SEC
        0x00, // MIN
        0x00, // HOUR
        0x40, // WEEK
        0x01, // DAY
        0x01, // MONTH
        0x00, // YEAR
        0x00, // RAM
        0x00, // AL_MIN
        0x00, // AL_HOUR
        0x00, // AL_WK_D
        0x00, // TIM0
        0x00, // TIM1
        0x00, // EXT
        0x00, // FLAG
        0x40, // CTRL
    };

    // reinit all
    _wire.beginTransmission(ADDRESS);
    for (uint8_t i = 0; i < sizeof(init_regs); ++i) {
      _wire.write(pgm_read_byte(init_regs + i));
    }
    _wire.endTransmission();
  }

  return true;
}

uint8_t RX8025T::readReg(RegAddr addr) {
  return i2c_rtc_read(_wire, ADDRESS, addr);
}

void RX8025T::writeReg(RegAddr addr, uint8_t val) {
  i2c_rtc_write(_wire, ADDRESS, addr, val);
}

void RX8025T::getTime(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_SEC);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t {7});
  timeptr->tm_sec = bcd2bin(_wire.read() & 0x7f);
  timeptr->tm_min = bcd2bin(_wire.read() & 0x7f);
  timeptr->tm_hour = bcd2bin(_wire.read() & 0x3f);
  timeptr->tm_wday = __builtin_ctz(_wire.read());
  timeptr->tm_mday = bcd2bin(_wire.read() & 0x3f);
  timeptr->tm_mon = bcd2bin(_wire.read() & 0x1f) - 1;
  timeptr->tm_year = bcd2bin(_wire.read()) + 100;
}

void RX8025T::setTime(const tm *t) {
  const uint8_t write_buf[] {
      REG_SEC,
      bin2bcd(t->tm_sec),
      bin2bcd(t->tm_min),
      bin2bcd(t->tm_hour),
      static_cast<uint8_t>(1U << t->tm_wday),
      bin2bcd(t->tm_mday),
      bin2bcd(t->tm_mon + 1),
      bin2bcd(t->tm_year - 100),
  };

  _wire.beginTransmission(ADDRESS);
  _wire.write(write_buf, sizeof(write_buf));
  _wire.endTransmission();
}

bool RX8025T::isRunning() {
  return (readReg(REG_CTRL) & 0x01) == 0;
}

void RX8025T::setRunning(bool running) {
  MASK_BOOL_REG_BITS(REG_CTRL, 0x01, !running);
}

RX8025T::TempCompIntv RX8025T::getTempCompInterval() {
  return static_cast<TempCompIntv>(readReg(REG_CTRL) & 0xc0);
}

void RX8025T::setTempCompIntv(TempCompIntv interval) {
  writeReg(REG_CTRL, (readReg(REG_CTRL) & 0x3f) | interval);
}

uint8_t RX8025T::getRAM() {
  return readReg(REG_RAM);
}

void RX8025T::setRAM(uint8_t val) {
  writeReg(REG_RAM, val);
}

RX8025T::TimerFreq RX8025T::getTimerFreq() {
  uint8_t ext = readReg(REG_EXT);

  if ((ext & 0x10) == 0) {
    // TE bit is 0
    return TF_OFF;
  } else {
    return static_cast<TimerFreq>(ext & 0x03);
  }
}

void RX8025T::setTimerFreq(TimerFreq freq) {
  if (freq == TF_OFF) {
    MASK_BOOL_REG_BITS(REG_EXT, 0x10, 0);
  } else {
    writeReg(REG_EXT, (readReg(REG_EXT) & 0xfc) | freq);
  }
}

bool RX8025T::isTimerIntrEnabled() {
  return (readReg(REG_CTRL) & 0x10) != 0;
}

void RX8025T::setTimerIntrEnabled(bool enabled) {
  MASK_BOOL_REG_BITS(REG_CTRL, 0x10, enabled);
}

bool RX8025T::getTimerFlag() {
  return (readReg(REG_FLAG) & 0x10) != 0;
}

void RX8025T::clearTimerFlag() {
  MASK_BOOL_REG_BITS(REG_FLAG, 0x10, 0);
}

RX8025T::FOUTFreq RX8025T::getFOUT() {
  uint8_t freq = readReg(REG_CTRL) & 0x0c;
  if (freq == 0x0c) {
    // 2'b11 is also 32768Hz
    return FOUT_32768HZ;
  } else {
    return static_cast<FOUTFreq>(freq);
  }
}

void RX8025T::setFOUT(FOUTFreq freq) {
  writeReg(REG_CTRL, (readReg(REG_CTRL) & 0xf3) | freq);
}

bool RX8025T::getVLF() {
  return (readReg(REG_FLAG) & 0x02) != 0;
}

void RX8025T::clearVLF() {
  MASK_BOOL_REG_BITS(REG_FLAG, 0x02, 0);
}

bool RX8025T::getVDET() {
  return (readReg(REG_FLAG) & 0x01) != 0;
}

void RX8025T::clearVDET() {
  MASK_BOOL_REG_BITS(REG_FLAG, 0x01, 0);
}

bool RX8025T::getUpdateFlag() {
  return (readReg(REG_FLAG) & 0x20) != 0;
}

void RX8025T::clearUpdateFlag() {
  MASK_BOOL_REG_BITS(REG_FLAG, 0x20, 0);
}

bool RX8025T::getUSEL() {
  return (readReg(REG_EXT) & 0x20) != 0;
}

void RX8025T::setUSEL(bool usel) {
  MASK_BOOL_REG_BITS(REG_EXT, 0x20, usel);
}

uint16_t RX8025T::getTimer() {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_TIM0);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t {2});
  uint16_t val = _wire.read();
  val |= _wire.read() << 8;
  return val;
}

void RX8025T::setTimer(uint16_t val) {
  const uint8_t write_buf[] {
      REG_TIM0,
      uint8_t(val & 0xff),
      uint8_t(val >> 8),
  };

  _wire.beginTransmission(ADDRESS);
  _wire.write(write_buf, sizeof(write_buf));
  _wire.endTransmission();
}

void RX8025T::getAlarm(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_AL_MIN);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t {6});
  uint8_t min = _wire.read();
  uint8_t hour = _wire.read();
  uint8_t day = _wire.read();
  _wire.read(); // Timer/Counter 0
  _wire.read(); // Timer/Counter 1
  uint8_t ext = _wire.read();

  bool wada = ext & 0x40;

  timeptr->tm_min = (min & 0x80) ? -1 : bcd2bin(min & 0x7f);
  timeptr->tm_hour = (hour & 0x80) ? -1 : bcd2bin(hour & 0x3f);

  if (day & 0x80) {
    timeptr->tm_wday = -1;
    timeptr->tm_mday = -1;
  } else if (wada) {
    timeptr->tm_wday = -1;
    timeptr->tm_mday = bcd2bin(day & 0x3f);
  } else {
    timeptr->tm_wday = day;
    timeptr->tm_mday = -1;
  }
}

void RX8025T::setAlarm(const tm *timeptr) {
  uint8_t min = (timeptr->tm_min == -1) ? 0x80 : bin2bcd(timeptr->tm_min);
  uint8_t hour = (timeptr->tm_hour == -1) ? 0x80 : bin2bcd(timeptr->tm_hour);
  uint8_t day = timeptr->tm_mday;
  uint8_t wday = timeptr->tm_wday;
  bool wada = false;

  if ((day == -1 && wday == -1) || (day != -1 && wday != -1)) {
    // does not match DAY/WEEK
    day = 0x80;
  } else if (day != -1) {
    // sets DAY as target of alarm function
    day = bin2bcd(day & 0x3f);
    wada = true;
  } else {
    // sets WEEK as target of alarm function
    day = wday & 0x7f;
  }

  const uint8_t write_buf[] {
      REG_AL_MIN,
      min,
      hour,
      day,
  };

  _wire.beginTransmission(ADDRESS);
  _wire.write(write_buf, sizeof(write_buf));
  _wire.endTransmission();

  if ((day & 0x80) == 0) {
    MASK_BOOL_REG_BITS(REG_EXT, 0x40, wada);
  }
}

bool RX8025T::isAlarmIntrEnabled() {
  return (readReg(REG_CTRL) & 0x08) != 0;
}

void RX8025T::setAlarmIntrEnabled(bool enabled) {
  MASK_BOOL_REG_BITS(REG_CTRL, 0x08, enabled);
}

bool RX8025T::getAlarmFlag() {
  return (readReg(REG_FLAG) & 0x08) != 0;
}

void RX8025T::clearAlarmFlag() {
  MASK_BOOL_REG_BITS(REG_FLAG, 0x08, 0);
}

PCF8563::PCF8563(TwoWire &wire) : _wire {wire} {}

bool PCF8563::setup() {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_VL_SEC);
  if (_wire.endTransmission() != 0) {
    return false;
  }

  _wire.requestFrom(ADDRESS, uint8_t {1});
  uint8_t vl = _wire.read();
  _wire.endTransmission();

  static constexpr uint8_t PROGMEM init_regs[] {
      REG_CTRL_1,
      0x00, // Control_status_1
      0x00, // Control_status_2
      0x00, // VL_seconds
      0x00, // Minutes
      0x00, // Hours
      0x01, // Days
      0x05, // Weekdays
      0x01, // Century_months
      0x00, // Years
  };
  const uint8_t bytes_to_write = (vl & 0x80) ? sizeof(init_regs) : 2;

  _wire.beginTransmission(ADDRESS);
  for (uint8_t i = 0; i < bytes_to_write; ++i) {
    _wire.write(pgm_read_byte(init_regs + i));
  }
  _wire.endTransmission();

  return true;
}

uint8_t PCF8563::readReg(RegAddr addr) {
  return i2c_rtc_read(_wire, ADDRESS, addr);
}

void PCF8563::writeReg(RegAddr addr, uint8_t val) {
  i2c_rtc_write(_wire, ADDRESS, addr, val);
}

void PCF8563::getTime(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_VL_SEC);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t {7});
  timeptr->tm_sec = bcd2bin(_wire.read() & 0x7f);
  timeptr->tm_min = bcd2bin(_wire.read() & 0x7f);
  timeptr->tm_hour = bcd2bin(_wire.read() & 0x3f);
  timeptr->tm_mday = bcd2bin(_wire.read() & 0x3f);
  timeptr->tm_wday = bcd2bin(_wire.read() & 0x07);
  uint8_t cen_mon = _wire.read();
  timeptr->tm_mon = bcd2bin(cen_mon & 0x1f) - 1;
  timeptr->tm_year = bcd2bin(_wire.read()) + 100;

  if (cen_mon & 0x80) {
    // century bit set
    cen_mon &= 0x7f;
    timeptr->tm_year += 100;
  }
}

void PCF8563::setTime(const tm *timeptr) {
  uint8_t year = timeptr->tm_year - 100;
  uint8_t cen_mon = bin2bcd(timeptr->tm_mon + 1);

  if (year >= 100) {
    cen_mon |= 0x80;
    year -= 100;
  }

  const uint8_t write_buf[] {
      REG_VL_SEC,
      bin2bcd(timeptr->tm_sec),
      bin2bcd(timeptr->tm_min),
      bin2bcd(timeptr->tm_hour),
      bin2bcd(timeptr->tm_mday),
      bin2bcd(timeptr->tm_wday),
      cen_mon,
      bin2bcd(year),
  };

  _wire.beginTransmission(ADDRESS);
  _wire.write(write_buf, sizeof(write_buf));
  _wire.endTransmission();
}

bool PCF8563::isRunning() {
  return (readReg(REG_CTRL_1) & 0x20) == 0;
}

void PCF8563::setRunning(bool running) {
  MASK_BOOL_REG_BITS(REG_CTRL_1, 0x20, !running);
}

PCF8563::CLKFreq PCF8563::getCLKOut() {
  uint8_t clkout = readReg(REG_CLKOUT);
  return (clkout & 0x80) ? static_cast<CLKFreq>(clkout & 0x07) : CLKOUT_OFF;
}

void PCF8563::setCLKOut(CLKFreq freq) {
  writeReg(REG_CLKOUT, freq);
}

uint8_t PCF8563::getTimer() {
  return readReg(REG_TIM);
}

void PCF8563::setTimer(uint8_t val) {
  writeReg(REG_TIM, val);
}

PCF8563::TimerFreq PCF8563::getTimerFreq() {
  uint8_t tim_ctrl = readReg(REG_TIM_CTRL);
  return (tim_ctrl & 0x80) ? TimerFreq {tim_ctrl} : TF_OFF;
}

void PCF8563::setTimerFreq(TimerFreq freq) {
  writeReg(REG_TIM_CTRL, freq);
}

bool PCF8563::isTimerIntrEnabled() {
  return (readReg(REG_CTRL_2) & 0x01) != 0;
}

void PCF8563::setTimerIntrEnabled(bool enabled) {
  MASK_BOOL_REG_BITS(REG_CTRL_2, 0x01, enabled);
}

bool PCF8563::getTimerFlag() {
  return (readReg(REG_CTRL_2) & 0x04) != 0;
}

void PCF8563::clearTimerFlag() {
  MASK_BOOL_REG_BITS(REG_CTRL_2, 0x04, 0);
}

bool PCF8563::isTimerPulseMode() {
  return (readReg(REG_CTRL_2) & 0x10) != 0;
}

void PCF8563::setTimerPulseMode(bool pulse_mode) {
  MASK_BOOL_REG_BITS(REG_CTRL_2, 0x10, pulse_mode);
}

void PCF8563::getAlarm(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_AL_MIN);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t {4});
  uint8_t min = _wire.read();
  uint8_t hour = _wire.read();
  uint8_t day = _wire.read();
  uint8_t wday = _wire.read();

  timeptr->tm_min = (min & 0x80) ? -1 : bcd2bin(min & 0x7f);
  timeptr->tm_hour = (hour & 0x80) ? -1 : bcd2bin(hour & 0x3f);
  timeptr->tm_mday = (day & 0x80) ? -1 : bcd2bin(day & 0x3f);
  timeptr->tm_wday = (wday & 0x80) ? -1 : (wday & 0x07);
}

void PCF8563::setAlarm(const tm *timeptr) {
  uint8_t min = (timeptr->tm_min == -1) ? 0x80 : bin2bcd(timeptr->tm_min);
  uint8_t hour = (timeptr->tm_hour == -1) ? 0x80 : bin2bcd(timeptr->tm_hour);
  uint8_t day = (timeptr->tm_mday == -1) ? 0x80 : bin2bcd(timeptr->tm_mday);
  uint8_t wday = (timeptr->tm_wday == -1) ? 0x80 : timeptr->tm_wday;

  const uint8_t write_buf[] {
      REG_AL_MIN,
      min,
      hour,
      day,
      wday,
  };

  _wire.beginTransmission(ADDRESS);
  _wire.write(write_buf, sizeof(write_buf));
  _wire.endTransmission();
}

bool PCF8563::isAlarmIntrEnabled() {
  return (readReg(REG_CTRL_2) & 0x02) != 0;
}

void PCF8563::setAlarmIntrEnabled(bool enabled) {
  MASK_BOOL_REG_BITS(REG_CTRL_2, 0x02, enabled);
}

bool PCF8563::getAlarmFlag() {
  return (readReg(REG_CTRL_2) & 0x08) != 0;
}

void PCF8563::clearAlarmFlag() {
  MASK_BOOL_REG_BITS(REG_CTRL_2, 0x08, 0);
}
