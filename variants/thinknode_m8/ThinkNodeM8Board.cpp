#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#ifdef QSPIFLASH
#include <CustomLFS_QSPIFlash.h>
#endif

#include "ThinkNodeM8Board.h"

#ifdef THINKNODE_M8

void ThinkNodeM8Board::begin() {
  NRF52Board::begin();

  Wire.begin();

#ifdef P_LORA_TX_LED
  pinMode(P_LORA_TX_LED, OUTPUT);
  digitalWrite(P_LORA_TX_LED, LOW);
#endif

  pinMode(PIN_CHARGE_STATUS, INPUT); // CHGLED has an external network, no output drive.

#ifdef QSPIFLASH
  // Also recover a retained Flash if the optional rail bypass is fitted.
  digitalWrite(PIN_QSPI_CS, HIGH);
  pinMode(PIN_QSPI_CS, OUTPUT);
  digitalWrite(PIN_QSPI_CS, LOW);
  delayMicroseconds(1);
  digitalWrite(PIN_QSPI_CS, HIGH);
  delayMicroseconds(50); // MX25R1635F tRDP <=45 us, before QSPIFlash.begin().
#endif
}

void ThinkNodeM8Board::shutdownPeripherals() {
  // Display sleep, LoRa cold sleep and M8 GPS backup while their buses still run.
  NRF52Board::shutdownPeripherals();

  if (Serial1) Serial1.end();
  Wire.end();
  SPI.end();
#ifdef DISPLAY_CLASS
  SPI1.end();
#endif
#ifdef PIN_BUZZER
  noTone(PIN_BUZZER);
  digitalWrite(PIN_BUZZER, LOW);
#endif
  digitalWrite(DISP_BACKLIGHT, LOW);

#ifdef QSPIFLASH
  if (QSPIFlash._qspi_initialized) {
    QSPIFlash.end(); // Unmount and finish filesystem writes before cutting the rail.
    if (QSPIFlash.waitForQSPIReady()) {
      nrf_qspi_cinstr_conf_t sleep_cmd = {};
      sleep_cmd.opcode = 0xB9;
      sleep_cmd.length = NRF_QSPI_CINSTR_LEN_1B;
      sleep_cmd.io2_level = true;
      sleep_cmd.io3_level = true;
      sleep_cmd.wipwait = true;
      if (nrfx_qspi_cinstr_xfer(&sleep_cmd, NULL, NULL) == NRFX_SUCCESS) {
        delayMicroseconds(10);
      }
    }
    nrfx_qspi_uninit();
    QSPIFlash._qspi_initialized = false;
  }
  // Include CS: a HIGH signal would back-power Flash after GPIO13 cuts its rail.
  const uint8_t flash_pins[] = {
    PIN_QSPI_CS, PIN_QSPI_SCK, PIN_QSPI_IO0, PIN_QSPI_IO1, PIN_QSPI_IO2, PIN_QSPI_IO3
  };
  for (uint8_t pin : flash_pins) nrf_gpio_cfg_default(pin);
#endif

  detachInterrupt(digitalPinToInterrupt(P_LORA_DIO_1));
  detachInterrupt(digitalPinToInterrupt(PIN_BUTTON1_A));
  detachInterrupt(digitalPinToInterrupt(PIN_BUTTON1_B));
  const uint8_t peripheral_pins[] = {
    PIN_SERIAL1_RX, PIN_SERIAL1_TX, PIN_GPS_PPS,
    PIN_WIRE_SDA, PIN_WIRE_SCL, PIN_BUTTON1_A, PIN_BUTTON1_B, PIN_BUTTON2,
    PIN_DISPLAY_CS, PIN_DISPLAY_DC, PIN_DISPLAY_RST, PIN_DISPLAY_BUSY,
    PIN_SPI1_SCK, PIN_SPI1_MOSI, P_LORA_SCLK, P_LORA_MOSI, P_LORA_MISO,
    P_LORA_DIO_1, P_LORA_BUSY, PIN_CHARGE_STATUS, 38, 15
  };
  for (uint8_t pin : peripheral_pins) nrf_gpio_cfg_default(pin);

  // Release signals before removing switchable rails; retain radio CS/reset HIGH.
  digitalWrite(DISP_EN, LOW);
  digitalWrite(PIN_PWR_EN, LOW);
  digitalWrite(PIN_GPS_EN, LOW);
  digitalWrite(PIN_GPS_RESET, LOW); // Q16 must not draw base current in SYSTEMOFF.
  digitalWrite(PIN_GPS_SLEEP_INT, HIGH);
  digitalWrite(SX126X_ANT_SW, LOW);
  digitalWrite(ADC_EN, LOW);

  // Keep the accepted single-press hardware wake path, without polling/RTC wake.
  nrf_gpio_cfg_sense_input(PIN_BUTTON1, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
}

uint16_t ThinkNodeM8Board::getBattMilliVolts() {
  int adcvalue = 0;

  digitalWrite(ADC_EN, HIGH);
  analogReference(AR_INTERNAL_2_4);
  analogReadResolution(12);
  delay(10);

  // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
  adcvalue = analogRead(PIN_VBAT_READ);
  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  digitalWrite(ADC_EN, LOW);
  return (uint16_t)((float)adcvalue * REAL_VBAT_MV_PER_LSB);
}
#endif
