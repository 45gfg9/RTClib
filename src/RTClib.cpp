#include "RTClib.h"

namespace __rtclib_details {
  // RAII class for data transferring to/from DS1302
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

  enum DS1302RegAddr : uint8_t {
    DS1302_W_SEC = 0x80,
    DS1302_R_SEC = 0x81,
    DS1302_W_MIN = 0x82,
    DS1302_R_MIN = 0x83,
    DS1302_W_HR = 0x84,
    DS1302_R_HR = 0x85,
    DS1302_W_DATE = 0x86,
    DS1302_R_DATE = 0x87,
    DS1302_W_MON = 0x88,
    DS1302_R_MON = 0x89,
    DS1302_W_DOW = 0x8a,
    DS1302_R_DOW = 0x8b,
    DS1302_W_YEAR = 0x8c,
    DS1302_R_YEAR = 0x8d,
    DS1302_W_WP = 0x8e,
    DS1302_R_WP = 0x8f,
    DS1302_W_TC = 0x90,
    DS1302_R_TC = 0x91,
    DS1302_W_CLKBURST = 0xbe,
    DS1302_R_CLKBURST = 0xbf,
    DS1302_W_RAM = 0xc0,
    DS1302_R_RAM = 0xc1,
    DS1302_W_RAMBURST = 0xfe,
    DS1302_R_RAMBURST = 0xff,
  };

  enum DS1307RegAddr : uint8_t {
    DS1307_SEC = 0x00,
    DS1307_MIN = 0x01,
    DS1307_HR = 0x02,
    DS1307_DOW = 0x03,
    DS1307_DATE = 0x04,
    DS1307_MON = 0x05,
    DS1307_YEAR = 0x06,
    DS1307_CTRL = 0x07,
    DS1307_RAM = 0x08,
  };

  enum DS3231RegAddr : uint8_t {
    DS3231_SEC = 0x00,
    DS3231_MIN = 0x01,
    DS3231_HR = 0x02,
    DS3231_DOW = 0x03,
    DS3231_DATE = 0x04,
    DS3231_MON = 0x05,
    DS3231_YEAR = 0x06,
    DS3231_AL1_SEC = 0x07,
    DS3231_AL1_MIN = 0x08,
    DS3231_AL1_HR = 0x09,
    DS3231_AL1_DATE = 0x0a,
    DS3231_AL2_MIN = 0x0b,
    DS3231_AL2_HR = 0x0c,
    DS3231_AL2_DATE = 0x0d,
    DS3231_CTRL = 0x0e,
    DS3231_STATUS = 0x0f,
    DS3231_AGING = 0x10,
    DS3231_TEMP_MSB = 0x11,
    DS3231_TEMP_LSB = 0x12,
  };

  enum RX8025TRegAddr : uint8_t {
    RX8025T_SEC = 0x00,
    RX8025T_MIN = 0x01,
    RX8025T_HOUR = 0x02,
    RX8025T_WEEK = 0x03,
    RX8025T_DAT = 0x04,
    RX8025T_MONTH = 0x05,
    RX8025T_YEAR = 0x06,
    RX8025T_RAM = 0x07,
    RX8025T_AL_MIN = 0x08,
    RX8025T_AL_HOUR = 0x09,
    RX8025T_AL_WK_D = 0x0a,
    RX8025T_TIM0 = 0x0b,
    RX8025T_TIM1 = 0x0c,
    RX8025T_EXT = 0x0d,
    RX8025T_FLAG = 0x0e,
    RX8025T_CTRL = 0x0f,
  };
} // namespace __rtclib_details

using namespace __rtclib_details;

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

  wire.requestFrom(dev, uint8_t(1));
  return wire.read();
}

#define MASK_BOOL_REG_BITS(reg, maskbits, boolval)  \
  do {                                              \
    uint8_t mask = (boolval) ? (maskbits) : 0;      \
    uint8_t regval = readReg(reg);                  \
    if ((regval & (maskbits)) != mask) {            \
      writeReg(reg, (regval & ~(maskbits)) | mask); \
    }                                               \
  } while (0)

DS1302::DS1302(uint8_t ce, uint8_t sck, uint8_t io) : _ce(ce), _sck(sck), _io(io) {}

bool DS1302::setup() {
  pinMode(_ce, OUTPUT);
  pinMode(_sck, OUTPUT);
  writeReg(DS1302_W_WP, 0);
  setRunning(true);

  return true;
}

uint8_t DS1302::_read() {
  pinMode(_io, INPUT);

  // shiftIn() will not work

  uint8_t value = 0;
  for (uint8_t i = 8; i; --i) {
    uint8_t bit = digitalRead(_io);
    value = (value >> 1) | (bit ? 0x80 : 0); // LSB first
    digitalWrite(_sck, HIGH);
    digitalWrite(_sck, LOW);
  }
  return value;
}

void DS1302::_write(uint8_t val) {
  pinMode(_io, OUTPUT);

  // shiftOut() will not work

  for (uint8_t i = 8; i; --i) {
    digitalWrite(_io, val & 1);
    val >>= 1;
    digitalWrite(_sck, HIGH);
    delayMicroseconds(1);
    digitalWrite(_sck, LOW);
    delayMicroseconds(1);
  }
}

uint8_t DS1302::readReg(uint8_t addr) {
  TransferHelper _tr(_ce, _sck);

  _write(addr);
  return _read();
}

void DS1302::writeReg(uint8_t addr, uint8_t val) {
  TransferHelper _tr(_ce, _sck);

  _write(addr);
  _write(val);
}

void DS1302::getTime(tm *timeptr) {
  TransferHelper _tr(_ce, _sck);

  _write(DS1302_R_CLKBURST);
  timeptr->tm_sec = bcd2bin(_read() & 0x7f);
  timeptr->tm_min = bcd2bin(_read());
  timeptr->tm_hour = bcd2bin(_read());
  timeptr->tm_mday = bcd2bin(_read());
  timeptr->tm_mon = bcd2bin(_read()) - 1;
  timeptr->tm_wday = _read();
  timeptr->tm_year = bcd2bin(_read()) + 100;

  if (timeptr->tm_wday == 7) {
    // Sunday
    timeptr->tm_wday = 0;
  }
}

void DS1302::setTime(const tm *timeptr) {
  TransferHelper _tr(_ce, _sck);

  uint8_t wday = timeptr->tm_wday;
  if (wday == 0) {
    // Sunday
    wday = 7;
  }

  _write(DS1302_W_CLKBURST);
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
  return (readReg(DS1302_R_SEC) & 0x80) == 0;
}

void DS1302::setRunning(bool running) {
  MASK_BOOL_REG_BITS(DS1302_R_SEC, 0x80, !running);
}

DS1302::trickle_charger_t DS1302::getTrickleCharger() {
  uint8_t r = readReg(DS1302_R_TC);

  // we need this because the register value might not always be valid
  if ((r & 0xf0) == 0xa0 && (r & 0x0c) != 0 && (r & 0x03) != 0x03) {
    return static_cast<trickle_charger_t>(r);
  }
  return TC_OFF;
}

void DS1302::setTrickleCharger(trickle_charger_t mode) {
  writeReg(DS1302_W_TC, mode);
}

uint8_t DS1302::readRAM(uint8_t index) {
  if (index >= RAM_SIZE) {
    return 0;
  }

  TransferHelper _tr(_ce, _sck);

  _write(DS1302_R_RAM + (index << 1));
  return _read();
}

void DS1302::writeRAM(uint8_t index, uint8_t val) {
  if (index >= RAM_SIZE) {
    return;
  }

  TransferHelper _tr(_ce, _sck);

  _write(DS1302_W_RAM + (index << 1));
  _write(val);
}

DS1307::DS1307(TwoWire &wire) : _wire(wire) {}

bool DS1307::setup() {
  _wire.beginTransmission(ADDRESS);
  return _wire.endTransmission() == 0;
}

uint8_t DS1307::readReg(uint8_t addr) {
  return i2c_rtc_read(_wire, ADDRESS, addr);
}

void DS1307::writeReg(uint8_t addr, uint8_t val) {
  i2c_rtc_write(_wire, ADDRESS, addr, val);
}

uint8_t DS1307::readRAM(uint8_t index) {
  if (index >= RAM_SIZE) {
    return 0;
  }

  return i2c_rtc_read(_wire, ADDRESS, DS1307_RAM + index);
}

void DS1307::writeRAM(uint8_t index, uint8_t val) {
  if (index >= RAM_SIZE) {
    return;
  }

  i2c_rtc_write(_wire, ADDRESS, DS1307_RAM + index, val);
}

void DS1307::getTime(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(DS1307_SEC);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t(7));
  timeptr->tm_sec = bcd2bin(_wire.read() & 0x7f);
  timeptr->tm_min = bcd2bin(_wire.read());
  timeptr->tm_hour = bcd2bin(_wire.read());
  timeptr->tm_wday = _wire.read();
  timeptr->tm_mday = bcd2bin(_wire.read());
  timeptr->tm_mon = bcd2bin(_wire.read()) - 1;
  timeptr->tm_year = bcd2bin(_wire.read()) + 100;

  if (timeptr->tm_wday == 7) {
    // Sunday
    timeptr->tm_wday = 0;
  }
}

void DS1307::setTime(const tm *timeptr) {
  uint8_t wday = timeptr->tm_wday;
  if (wday == 0) {
    // Sunday
    wday = 7;
  }

  _wire.beginTransmission(ADDRESS);
  _wire.write(DS1307_SEC);
  _wire.write(bin2bcd(timeptr->tm_sec));
  _wire.write(bin2bcd(timeptr->tm_min));
  _wire.write(bin2bcd(timeptr->tm_hour));
  _wire.write(wday);
  _wire.write(bin2bcd(timeptr->tm_mday));
  _wire.write(bin2bcd(timeptr->tm_mon + 1));
  _wire.write(bin2bcd(timeptr->tm_year - 100));
  _wire.endTransmission();
}

bool DS1307::isRunning() {
  return (readReg(DS1307_SEC) & 0x80) == 0;
}

void DS1307::setRunning(bool running) {
  MASK_BOOL_REG_BITS(DS1307_SEC, 0x80, !running);
}

DS1307::sqw_out_t DS1307::getSQWOut() {
  uint8_t r = readReg(DS1307_CTRL);

  if (r & 0x10) {
    // SQWE set
    return static_cast<sqw_out_t>(r & 0x7f);
  }

  return static_cast<sqw_out_t>(r & 0xfc);
}

void DS1307::setSQWOut(sqw_out_t value) {
  writeReg(DS1307_CTRL, value);
}

DS3231::DS3231(TwoWire &wire) : _wire(wire) {}

bool DS3231::setup() {
  _wire.beginTransmission(ADDRESS);
  return _wire.endTransmission() == 0;
}

uint8_t DS3231::readReg(uint8_t addr) {
  return i2c_rtc_read(_wire, ADDRESS, addr);
}

void DS3231::writeReg(uint8_t addr, uint8_t val) {
  i2c_rtc_write(_wire, ADDRESS, addr, val);
}

void DS3231::getTime(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(DS3231_SEC);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t(7));
  timeptr->tm_sec = bcd2bin(_wire.read() & 0x7f);
  timeptr->tm_min = bcd2bin(_wire.read());
  timeptr->tm_hour = bcd2bin(_wire.read());
  timeptr->tm_wday = _wire.read();
  timeptr->tm_mday = bcd2bin(_wire.read());
  uint8_t cen_mon = _wire.read();
  timeptr->tm_mon = bcd2bin(cen_mon & 0x1f) - 1;
  timeptr->tm_year = bcd2bin(_wire.read()) + 100;

  if (cen_mon & 0x80) {
    // century bit set
    timeptr->tm_year += 100;
  }

  if (timeptr->tm_wday == 7) {
    // Sunday
    timeptr->tm_wday = 0;
  }
}

void DS3231::setTime(const tm *timeptr) {
  uint8_t wday = timeptr->tm_wday;
  if (wday == 0) {
    // Sunday
    wday = 7;
  }

  uint8_t year = timeptr->tm_year - 100;
  uint8_t cen_mon = bin2bcd(timeptr->tm_mon + 1);

  if (year >= 100) {
    cen_mon |= 0x80;
    year -= 100;
  }

  _wire.beginTransmission(ADDRESS);
  _wire.write(DS3231_SEC);
  _wire.write(bin2bcd(timeptr->tm_sec));
  _wire.write(bin2bcd(timeptr->tm_min));
  _wire.write(bin2bcd(timeptr->tm_hour));
  _wire.write(wday);
  _wire.write(bin2bcd(timeptr->tm_mday));
  _wire.write(cen_mon);
  _wire.write(bin2bcd(year));
  _wire.endTransmission();
}

bool DS3231::isRunning() {
  return (readReg(DS3231_CTRL) & 0x80) == 0;
}

void DS3231::setRunning(bool running) {
  MASK_BOOL_REG_BITS(DS3231_SEC, 0x80, !running);
}

bool DS3231::getINTCN() {
  return (readReg(DS3231_CTRL) & 0x04) != 0;
}

void DS3231::setINTCN(bool intcn) {
  MASK_BOOL_REG_BITS(DS3231_CTRL, 0x04, intcn);
}

bool DS3231::getBBSQW() {
  return (readReg(DS3231_CTRL) & 0x40) != 0;
}

void DS3231::setBBSQW(bool bbsqw) {
  MASK_BOOL_REG_BITS(DS3231_CTRL, 0x40, bbsqw);
}

DS3231::sqw_t DS3231::getSQWFreq() {
  return static_cast<sqw_t>(readReg(DS3231_CTRL) & 0x18);
}

void DS3231::setSQWFreq(sqw_t freq) {
  uint8_t ctrl = readReg(DS3231_CTRL);
  writeReg(DS3231_CTRL, (ctrl & 0xe7) | freq);
}

bool DS3231::isIntrEnabled() {
  return (readReg(DS3231_CTRL) & 0x04) != 0;
}

void DS3231::setIntrEnabled(bool enabled) {
  MASK_BOOL_REG_BITS(DS3231_CTRL, 0x04, enabled);
}

bool DS3231::isAL1IntrEnabled() {
  return (readReg(DS3231_CTRL) & 0x01) != 0;
}

void DS3231::setAL1IntrEnabled(bool enabled) {
  MASK_BOOL_REG_BITS(DS3231_CTRL, 0x01, enabled);
}

bool DS3231::getAL1IntrFlag() {
  return (readReg(DS3231_STATUS) & 0x01) != 0;
}

void DS3231::clearAL1IntrFlag() {
  MASK_BOOL_REG_BITS(DS3231_STATUS, 0x01, 0);
}

bool DS3231::isAL2IntrEnabled() {
  return (readReg(DS3231_CTRL) & 0x02) != 0;
}

void DS3231::setAL2IntrEnabled(bool enabled) {
  MASK_BOOL_REG_BITS(DS3231_CTRL, 0x02, enabled);
}

bool DS3231::getAL2IntrFlag() {
  return (readReg(DS3231_STATUS) & 0x02) != 0;
}

void DS3231::clearAL2IntrFlag() {
  MASK_BOOL_REG_BITS(DS3231_STATUS, 0x02, 0);
}

DS3231::alarm_1_rate DS3231::getAL1(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(DS3231_AL1_SEC);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t(4));
  uint8_t sec = _wire.read();
  uint8_t min = _wire.read();
  uint8_t hr = _wire.read();
  uint8_t date = _wire.read();

  timeptr->tm_sec = bcd2bin(sec & 0x7f);
  timeptr->tm_min = bcd2bin(min & 0x7f);
  timeptr->tm_hour = bcd2bin(hr & 0x3f);

  bool dy_dt = date & 0x40;

  if (dy_dt) {
    // DY/#DT bit set, match day of week
    timeptr->tm_wday = date & 0x07;
    if (timeptr->tm_wday == 7) {
      // Sunday
      timeptr->tm_wday = 0;
    }
    timeptr->tm_mday = 0;
  } else {
    // DY/#DT bit clear, match date
    timeptr->tm_mday = bcd2bin(date & 0x3f);
    timeptr->tm_wday = 0;
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

void DS3231::setAL1(alarm_1_rate rate, const tm *timeptr) {
  uint8_t sec = bin2bcd(timeptr->tm_sec);
  uint8_t min = bin2bcd(timeptr->tm_min);
  uint8_t hr = bin2bcd(timeptr->tm_hour);
  uint8_t date = bin2bcd(timeptr->tm_mday);
  uint8_t wday = timeptr->tm_wday;
  if (wday == 0) {
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

  _wire.beginTransmission(ADDRESS);
  _wire.write(DS3231_AL1_SEC);
  _wire.write(sec);
  _wire.write(min);
  _wire.write(hr);
  _wire.write(date);
  _wire.endTransmission();
}

DS3231::alarm_2_rate DS3231::getAL2(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(DS3231_AL2_MIN);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t(3));
  uint8_t min = _wire.read();
  uint8_t hr = _wire.read();
  uint8_t date = _wire.read();

  timeptr->tm_min = bcd2bin(min & 0x7f);
  timeptr->tm_hour = bcd2bin(hr & 0x3f);

  bool dy_dt = date & 0x40;

  if (dy_dt) {
    // DY/#DT bit set, match day of week
    timeptr->tm_wday = date & 0x07;
    if (timeptr->tm_wday == 7) {
      // Sunday
      timeptr->tm_wday = 0;
    }
    timeptr->tm_mday = 0;
  } else {
    // DY/#DT bit clear, match date
    timeptr->tm_mday = bcd2bin(date & 0x3f);
    timeptr->tm_wday = 0;
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

void DS3231::setAL2(alarm_2_rate rate, const tm *timeptr) {
  uint8_t min = bin2bcd(timeptr->tm_min);
  uint8_t hr = bin2bcd(timeptr->tm_hour);
  uint8_t date = bin2bcd(timeptr->tm_mday);
  uint8_t wday = timeptr->tm_wday;
  if (wday == 0) {
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
      date |= 0x40;
      break;
  }

  _wire.beginTransmission(ADDRESS);
  _wire.write(DS3231_AL2_MIN);
  _wire.write(min);
  _wire.write(hr);
  _wire.write(date);
  _wire.endTransmission();
}

int8_t DS3231::getAgingOffset() {
  return static_cast<int8_t>(readReg(DS3231_AGING));
}

void DS3231::setAgingOffset(int8_t offset) {
  writeReg(DS3231_AGING, static_cast<uint8_t>(offset));
}

float DS3231::getTemperature() {
  _wire.beginTransmission(ADDRESS);
  _wire.write(DS3231_TEMP_MSB);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t(2));
  uint8_t msb = _wire.read();
  uint8_t lsb = _wire.read();

  int16_t temp = (msb << 8) | lsb;
  return temp / 256.0f;
}

RX8025T::RX8025T(TwoWire &wire) : _wire(wire) {}

bool RX8025T::setup() {
  _wire.beginTransmission(ADDRESS);
  _wire.write(RX8025T_FLAG);
  if (_wire.endTransmission() != 0) {
    return false;
  }

  _wire.requestFrom(ADDRESS, uint8_t(1));
  uint8_t flag = _wire.read();
  _wire.endTransmission();

  // check VLF
  if (flag & 0x02) {
    // reinit all
    _wire.beginTransmission(ADDRESS);
    _wire.write(RX8025T_SEC);
    _wire.write(0x00); // SEC
    _wire.write(0x00); // MIN
    _wire.write(0x00); // HOUR
    _wire.write(0x40); // WEEK
    _wire.write(0x01); // DAY
    _wire.write(0x01); // MONTH
    _wire.write(0x00); // YEAR
    _wire.write(0x00); // RAM
    _wire.write(0x00); // AL_MIN
    _wire.write(0x00); // AL_HOUR
    _wire.write(0x00); // AL_WK_D
    _wire.write(0x00); // TIM0
    _wire.write(0x00); // TIM1
    _wire.write(0x00); // EXT
    _wire.write(0x00); // FLAG
    _wire.write(0x40); // CTRL
    _wire.endTransmission();
  }

  return true;
}

uint8_t RX8025T::readReg(uint8_t addr) {
  return i2c_rtc_read(_wire, ADDRESS, addr);
}

void RX8025T::writeReg(uint8_t addr, uint8_t val) {
  i2c_rtc_write(_wire, ADDRESS, addr, val);
}

void RX8025T::getTime(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(RX8025T_SEC);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t(7));
  timeptr->tm_sec = bcd2bin(_wire.read() & 0x7f);
  timeptr->tm_min = bcd2bin(_wire.read() & 0x7f);
  timeptr->tm_hour = bcd2bin(_wire.read() & 0x3f);
  timeptr->tm_wday = __builtin_ctz(_wire.read());
  timeptr->tm_mday = bcd2bin(_wire.read() & 0x3f);
  timeptr->tm_mon = bcd2bin(_wire.read() & 0x1f) - 1;
  timeptr->tm_year = bcd2bin(_wire.read()) + 100;
}

void RX8025T::setTime(const tm *t) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(RX8025T_SEC);
  _wire.write(bin2bcd(t->tm_sec));
  _wire.write(bin2bcd(t->tm_min));
  _wire.write(bin2bcd(t->tm_hour));
  _wire.write(1U << t->tm_wday);
  _wire.write(bin2bcd(t->tm_mday));
  _wire.write(bin2bcd(t->tm_mon + 1));
  _wire.write(bin2bcd(t->tm_year - 100));
  _wire.endTransmission();
}

bool RX8025T::isRunning() {
  return (readReg(RX8025T_CTRL) & 0x01) == 0;
}

void RX8025T::setRunning(bool running) {
  MASK_BOOL_REG_BITS(RX8025T_CTRL, 0x01, !running);
}

RX8025T::temp_comp_intv RX8025T::getTempCompInterval() {
  return static_cast<temp_comp_intv>(readReg(RX8025T_CTRL) & 0xc0);
}

void RX8025T::setTempCompInterval(temp_comp_intv interval) {
  writeReg(RX8025T_CTRL, (readReg(RX8025T_CTRL) & 0x3f) | interval);
}

uint8_t RX8025T::getRAM() {
  return readReg(RX8025T_RAM);
}

void RX8025T::setRAM(uint8_t val) {
  writeReg(RX8025T_RAM, val);
}

RX8025T::timer_freq RX8025T::getTimerFreq() {
  uint8_t ext = readReg(RX8025T_EXT);

  if ((ext & 0x10) == 0) {
    // TE bit is 0
    return TF_OFF;
  } else {
    return static_cast<timer_freq>(ext & 0x03);
  }
}

void RX8025T::setTimerFreq(timer_freq freq) {
  if (freq == TF_OFF) {
    MASK_BOOL_REG_BITS(RX8025T_EXT, 0x10, 0);
  } else {
    writeReg(RX8025T_EXT, (readReg(RX8025T_EXT) & 0xfc) | freq);
  }
}

bool RX8025T::isTimerIntrEnabled() {
  return (readReg(RX8025T_CTRL) & 0x10) != 0;
}

void RX8025T::setTimerIntrEnabled(bool enabled) {
  MASK_BOOL_REG_BITS(RX8025T_CTRL, 0x10, enabled);
}

bool RX8025T::getTimerFlag() {
  return (readReg(RX8025T_FLAG) & 0x10) != 0;
}

void RX8025T::clearTimerFlag() {
  MASK_BOOL_REG_BITS(RX8025T_FLAG, 0x10, 0);
}

RX8025T::fout_freq RX8025T::getFOUT() {
  uint8_t freq = readReg(RX8025T_CTRL) & 0x0c;
  if (freq == 0x0c) {
    // 2'b11 is also 32768Hz
    return FOUT_32768HZ;
  } else {
    return static_cast<fout_freq>(freq);
  }
}

void RX8025T::setFOUT(fout_freq freq) {
  writeReg(RX8025T_CTRL, (readReg(RX8025T_CTRL) & 0xf3) | freq);
}

bool RX8025T::getVLF() {
  return (readReg(RX8025T_FLAG) & 0x02) != 0;
}

void RX8025T::clearVLF() {
  MASK_BOOL_REG_BITS(RX8025T_FLAG, 0x02, 0);
}

bool RX8025T::getVDET() {
  return (readReg(RX8025T_FLAG) & 0x01) != 0;
}

void RX8025T::clearVDET() {
  MASK_BOOL_REG_BITS(RX8025T_FLAG, 0x01, 0);
}

bool RX8025T::getUpdateFlag() {
  return (readReg(RX8025T_FLAG) & 0x20) != 0;
}

void RX8025T::clearUpdateFlag() {
  MASK_BOOL_REG_BITS(RX8025T_FLAG, 0x20, 0);
}

bool RX8025T::getUSEL() {
  return (readReg(RX8025T_EXT) & 0x20) != 0;
}

void RX8025T::setUSEL(bool usel) {
  MASK_BOOL_REG_BITS(RX8025T_EXT, 0x20, usel);
}

uint16_t RX8025T::getTimer() {
  _wire.beginTransmission(ADDRESS);
  _wire.write(RX8025T_TIM0);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t(2));
  uint16_t val = _wire.read();
  val |= _wire.read() << 8;
  return val;
}

void RX8025T::setTimer(uint16_t val) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(RX8025T_TIM0);
  _wire.write(val & 0xff);
  _wire.write(val >> 8);
  _wire.endTransmission();
}

void RX8025T::getAlarm(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(RX8025T_AL_MIN);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t(6));
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
    day = wday;
  }

  _wire.beginTransmission(ADDRESS);
  _wire.write(RX8025T_AL_MIN);
  _wire.write(min);
  _wire.write(hour);
  _wire.write(day);
  _wire.endTransmission();

  if ((day & 0x80) == 0) {
    MASK_BOOL_REG_BITS(RX8025T_EXT, 0x40, wada);
  }
}

bool RX8025T::isAlarmIntrEnabled() {
  return (readReg(RX8025T_CTRL) & 0x08) != 0;
}

void RX8025T::setAlarmIntrEnabled(bool enabled) {
  MASK_BOOL_REG_BITS(RX8025T_CTRL, 0x08, enabled);
}

bool RX8025T::getAlarmFlag() {
  return (readReg(RX8025T_FLAG) & 0x08) != 0;
}

void RX8025T::clearAlarmFlag() {
  MASK_BOOL_REG_BITS(RX8025T_FLAG, 0x08, 0);
}
