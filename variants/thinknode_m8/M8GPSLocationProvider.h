#pragma once

#include <helpers/sensors/MicroNMEALocationProvider.h>

// M8 V1.1: EN switches VCORE/RF only; VSYS/VRTC stay powered.
// Follow the official M8 Airoha backup-sleep/reset-wake sequence.
class M8GPSLocationProvider : public MicroNMEALocationProvider {
  Stream& _serial;

  bool enterBackup() {
    static const char ack[] = "$PAIR001,650,0";
    for (uint8_t attempt = 0; attempt < 2; ++attempt) {
      while (_serial.available()) _serial.read();
      digitalWrite(PIN_GPS_SLEEP_INT, LOW);
      delay(50);
      _serial.print("$PAIR650,0*25\r\n");
      _serial.flush();
      digitalWrite(PIN_GPS_SLEEP_INT, HIGH);

      uint8_t matched = 0;
      char previous = 0;
      bool nmea_seen = false;
      const unsigned long started = millis();
      while (millis() - started < 1200) { // 100 ms ACK + >1 Hz NMEA quiet window
        while (_serial.available() && millis() - started < 1200) {
          const char c = _serial.read();
          if (previous == '$' && c == 'G') nmea_seen = true;
          previous = c;
          matched = c == ack[matched] ? matched + 1 : (c == ack[0] ? 1 : 0);
          if (!ack[matched]) return true;
        }
        delay(1);
      }
      if (!nmea_seen) return true;
    }
    return false;
  }

public:
  M8GPSLocationProvider(Stream& serial, mesh::RTCClock* clock)
      : MicroNMEALocationProvider(serial, clock, -1), _serial(serial) {}

  void begin() override {
    digitalWrite(PIN_GPS_RESET, LOW);
    digitalWrite(PIN_GPS_SLEEP_INT, HIGH);
    MicroNMEALocationProvider::begin();
  }

  void reset() override {
    delay(50); // Let the switched 0.8V rail settle before asserting RESETB.
    digitalWrite(PIN_GPS_RESET, HIGH);
    delay(10);
    digitalWrite(PIN_GPS_RESET, LOW);
  }

  void stop() override {
    if (isEnabled()) {
      if (!enterBackup()) {
        MESH_DEBUG_PRINTLN("M8 GPS backup sleep not acknowledged; current requires verification");
      }
      delay(50); // Allow the receiver to park before removing VCORE/RF.
    }
    MicroNMEALocationProvider::stop();
    digitalWrite(PIN_GPS_RESET, LOW); // Never hold the inverting Q16 transistor on.
    digitalWrite(PIN_GPS_SLEEP_INT, HIGH);
  }
};
