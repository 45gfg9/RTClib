#include "RTClib.h"

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

static constexpr uint8_t bcd2bin(uint8_t val) {
  return val - 6 * (val >> 4);
}

static constexpr uint8_t bin2bcd(uint8_t val) {
  return val + 6 * (val / 10);
}

void DS1302::begin(uint8_t ce, uint8_t sck, uint8_t io) {
  pinMode((_ce = ce), OUTPUT);
  pinMode((_sck = sck), OUTPUT);
  pinMode((_io = io), INPUT);
  writeReg(DS1302_W_WP, 0);
  setRunning(true);
}

uint8_t DS1302::read() {
  // FIXME: this works while shiftIn() doesn't
  uint8_t value = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    uint8_t bit = digitalRead(_io);
    value |= (bit << i); // LSB first
    digitalWrite(_sck, HIGH);
    digitalWrite(_sck, LOW);
  }
  return value;
}

void DS1302::write(uint8_t val) {
  shiftOut(_io, _sck, LSBFIRST, val);
}

uint8_t DS1302::readReg(uint8_t addr) {
  TransferHelper _tr(_ce, _sck);

  write(addr);
  return read();
}

void DS1302::writeReg(uint8_t addr, uint8_t val) {
  TransferHelper _tr(_ce, _sck);

  write(addr);
  write(val);
}

void DS1302::getTime(tm *timeptr) {
  TransferHelper _tr(_ce, _sck);

  write(DS1302_R_CLKBURST);
  timeptr->tm_sec = bcd2bin(read() & 0x7f);
  timeptr->tm_min = bcd2bin(read());
  timeptr->tm_hour = bcd2bin(read());
  timeptr->tm_mday = bcd2bin(read());
  timeptr->tm_mon = bcd2bin(read()) - 1;
  timeptr->tm_wday = read();
  timeptr->tm_year = bcd2bin(read()) + 100;

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

  write(DS1302_W_CLKBURST);
  write(bin2bcd(timeptr->tm_sec));
  write(bin2bcd(timeptr->tm_min));
  write(bin2bcd(timeptr->tm_hour));
  write(bin2bcd(timeptr->tm_mday));
  write(bin2bcd(timeptr->tm_mon + 1));
  write(wday);
  write(bin2bcd(timeptr->tm_year - 100));
  write(0);
}

bool DS1302::isRunning() {
  return readReg(DS1302_R_SEC) & 0x80;
}

void DS1302::setRunning(bool running) {
  uint8_t sec = readReg(DS1302_R_SEC);
  if ((sec & 0x80) != (running << 7)) {
    writeReg(DS1302_W_SEC, (sec & 0x7f) | (running << 7));
  }
}

uint8_t DS1302::readRAM(uint8_t index) {
  if (index >= RAM_SIZE) {
    return 0xff;
  }

  TransferHelper _tr(_ce, _sck);

  write(DS1302_R_RAM + (index << 1));
  return read();
}

void DS1302::writeRAM(uint8_t index, uint8_t val) {
  if (index >= RAM_SIZE) {
    return;
  }

  TransferHelper _tr(_ce, _sck);

  write(DS1302_W_RAM + (index << 1));
  write(val);
}
