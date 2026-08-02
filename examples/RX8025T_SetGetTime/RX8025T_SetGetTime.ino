/*
 * Simple set/get example for an RX8025T RTC module.
 *
 * Note: RTClib.h documents that only the basic timekeeping functions of
 * RX8025T (used below) are stable; other functions (alarm, timer, RAM...)
 * are subject to change.
 *
 * On first boot (or after a power loss), it sets the clock to a fixed
 * date/time. On every loop, it reads the current time back and prints it.
 *
 * Wiring (I2C):
 *   VCC -> 5V (or 3.3V, depending on the module)
 *   GND -> GND
 *   SDA -> SDA (A4 on Uno)
 *   SCL -> SCL (A5 on Uno)
 */

#include <RTClib.h>

RX8025T rtc;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!rtc.setup()) {
    Serial.println("Couldn't find RTC, check wiring!");
    while (true) {}
  }

  if (!rtc.isRunning()) {
    Serial.println("RTC is not running, setting time...");

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
    rtc.setRunning(true);
  }
}

void loop() {
  tm now;
  rtc.getTime(&now);

  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &now);
  Serial.println(buf);

  delay(1000);
}
