#include "RTClib.h"

namespace __rtclib_details {
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

  if (false) {
    // FIXME: shiftIn() will not work
    return shiftIn(_io, _sck, LSBFIRST);
  }

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
  shiftOut(_io, _sck, LSBFIRST, val);
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
