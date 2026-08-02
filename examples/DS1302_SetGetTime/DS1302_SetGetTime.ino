/*
 * Simple set/get example for a DS1302 RTC module.
 *
 * Unlike the I2C RTCs (DS1307, DS3231, ...), the DS1302 has no way to tell
 * whether it lost power, so rtc.setup() always enables the oscillator and
 * there is no isRunning() check that reliably means "time needs setting".
 * Instead: upload this sketch once with SET_TIME defined to set the clock,
 * then comment out the #define below and upload again so the time isn't
 * reset every time the board resets/powers up.
 *
 * Wiring (3-wire):
 *   VCC -> 5V
 *   GND -> GND
 *   CE (RST) -> pin 4
 *   SCLK     -> pin 5
 *   I/O (DAT) -> pin 6
 */

#include <RTClib.h>

#define SET_TIME

DS1302 rtc(4, 5, 6); // CE, SCLK, I/O

void setup() {
  Serial.begin(115200);

  rtc.setup();

#ifdef SET_TIME
  Serial.println("Setting time...");

  // 2026-08-02 12:00:00, Sunday
  tm t {};
  t.tm_year = 2026 - 1900;
  t.tm_mon = 8 - 1;
  t.tm_mday = 2;
  t.tm_hour = 12;
  t.tm_min = 0;
  t.tm_sec = 0;
  t.tm_wday = 0;

  rtc.setTime(&t);
#endif
}

void loop() {
  tm now;
  rtc.getTime(&now);

  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &now);
  Serial.println(buf);

  delay(1000);
}
