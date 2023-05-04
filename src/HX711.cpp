/**
 *
 * HX711 library for Arduino
 * https://github.com/bogde/HX711
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 * (c) 2023 Vas
 *
**/

#include "HX711.h"

HX711* _theHX711;

HX711::HX711(TIM_TypeDef* timerInstance) {
  _theHX711 = this;

  _hx711ReadTimer = new HardwareTimer(timerInstance);
  _hx711ReadTimer->setOverflow(2, MICROSEC_FORMAT);
  _hx711ReadTimer->attachInterrupt(_onHX711ReadTimerInterrupt);
}

HX711::~HX711() {
}

void HX711::begin(uint32_t dout, uint32_t pd_sck, uint8_t gain) {
  PD_SCK = pd_sck;
  DOUT = dout;

  PD_SCK_PN = digitalPinToPinName(PD_SCK);
  DOUT_PN = digitalPinToPinName(DOUT);

  pinMode(PD_SCK, OUTPUT);
  pinMode(DOUT, INPUT);

  set_gain(gain);

  readStatus = 128u;
}

bool HX711::is_ready(void) {
  return readStatus & 32u;
}

void HX711::set_gain(uint8_t gain) {
  switch (gain) {
  case 128u:		// channel A, gain factor 128
    GAIN = 1u;
    break;
  case 64u:		// channel A, gain factor 64
    GAIN = 3u;
    break;
  case 32u:		// channel B, gain factor 32
    GAIN = 2u;
    break;
  }
}

long HX711::read(unsigned long timeout) {

  // Wait for the chip to become ready.
  if (!wait_ready_timeout(timeout)) {
    return 0;
  }

  return static_cast<long>(readData) - OFFSET;
}

void HX711::wait_ready(unsigned long delay_ms) {
  // Wait for the chip to become ready.
  // This is a blocking implementation and will
  // halt the sketch until a load cell is connected.
  while (!is_ready()) {
    // Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
    // https://github.com/bogde/HX711/issues/73
    delay(delay_ms);
  }
}

bool HX711::wait_ready_retry(unsigned int retries, unsigned long delay_ms) {
  // Wait for the chip to become ready by
  // retrying for a specified amount of attempts.
  // https://github.com/bogde/HX711/issues/76
  int count = 0;
  while (count < retries) {
    if (is_ready()) {
      return true;
    }
    delay(delay_ms);
    count++;
  }
  return false;
}

bool HX711::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
  // Wait for the chip to become ready until timeout.
  // https://github.com/bogde/HX711/pull/96
  unsigned long stopAt = millis() + timeout;
  while (millis() < stopAt) {
    if (is_ready()) {
      return true;
    }
    delay(delay_ms);
  }
  return false;
}

long HX711::read_average(const byte times) {
  if (times <= 1) {
    return read();
  }
  long sum = 0;
  for (byte i = 0; i < times; ++i) {
    sum += read();
  }
  return sum / times;
}

long HX711::get_value(const byte times) {
  return read_average(times);
}

float HX711::get_units(byte times) {
  return read_average(times) / (SCALE == 0 ? 1 : SCALE);
}

void HX711::tare(byte times) {
  set_offset(0);
  set_offset(read_average(times));
}

void HX711::set_scale(float scale) {
  SCALE = scale;
}

float HX711::get_scale(void) {
  return SCALE;
}

void HX711::set_offset(long offset) {
  OFFSET = offset;
}

long HX711::get_offset(void) {
  return OFFSET;
}

void HX711::power_down() {
  if (!(readStatus & 128u)) {
    _hx711ReadTimer->pause();
    readStatus = 128u;
    digitalWriteFast(PD_SCK_PN, HIGH);
  }
}

void HX711::power_up() {
  if (readStatus & 128u) {
    digitalWriteFast(PD_SCK_PN, LOW);
    readStatus = 0u;
    _hx711ReadTimer->refresh();
    _hx711ReadTimer->resume();
  }
}

void HX711::processReadTimerInterrupt(void) {
  if (!(readStatus & 63u)) {
    if (digitalReadFast(DOUT_PN)) { // conversion not ready
      return;
    }
  }

  uint8_t position = (readStatus >> 1) & 31u;

  if (readStatus & 1u) {
    if (position < 24u) { // 24 data bits
      if (position == 0u) {
        readBuffer = 0u;
      }

      if (digitalReadFast(DOUT_PN)) {
        readBuffer |= (1u << (23u - position));
      }

      if (position == 23u) { // all bits received
        // Replicate the most significant bit to pad out a 32-bit signed integer
        if (readBuffer & 0x800000) {
          readData = readBuffer | 0xFF000000;
        }
        else {
          readData = readBuffer;
        }
      }
    }
    digitalWriteFast(PD_SCK_PN, LOW);
  }
  else {
    if (position < 24u) { // 24 data bits
      digitalWriteFast(PD_SCK_PN, HIGH);
    }
    // set gain bits
    else if (position == 24u && GAIN > 0) {
      digitalWriteFast(PD_SCK_PN, HIGH);
    }
    else if (position == 25u && GAIN > 1) {
      digitalWriteFast(PD_SCK_PN, HIGH);
    }
    else if (position == 26u && GAIN > 2) {
      digitalWriteFast(PD_SCK_PN, HIGH);
    }
  }

  if (++readStatus > 127u) { // wrap around
    digitalWriteFast(PD_SCK_PN, LOW);
    readStatus -= 64u;
  }
}

void HX711::_onHX711ReadTimerInterrupt(void) {
  _theHX711->processReadTimerInterrupt();
}
