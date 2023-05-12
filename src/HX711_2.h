/**
 *
 * HX711 library for Arduino
 * https://github.com/bogde/HX711
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 *
**/
#pragma once

#include <Arduino.h>

class HX711_2
{
private:
  PinName PD_SCK_PN;
  PinName DOUT_PN;
  PinName DOUT2_PN;
  uint32_t PD_SCK;	// Power Down and Serial Clock Input Pin
  uint32_t DOUT;		// Serial Data Output Pin
  uint32_t DOUT2;		// Serial Data Output Pin
  uint8_t GAIN;		// amplification factor
  long OFFSET = 0;	// used for tare weight
  long OFFSET2 = 0;	// used for tare weight
  float SCALE = 1.f;	// used to return weight in grams, kg, ounces, whatever
  float SCALE2 = 1.f;	// used to return weight in grams, kg, ounces, whatever

  HardwareTimer* _hx711ReadTimer;
  volatile uint32_t readData[2];
  volatile uint32_t readBuffer[2];
  volatile uint32_t readStatus;

  static inline void _onHX711ReadTimerInterrupt(void);
  void processReadTimerInterrupt(void);

public:

  HX711_2(TIM_TypeDef* timerInstance = TIM10);

  virtual ~HX711_2();

  // Initialize library with data output pin, clock input pin and gain factor.
  // Channel selection is made by passing the appropriate gain:
  // - With a gain factor of 64 or 128, channel A is selected
  // - With a gain factor of 32, channel B is selected
  // The library default is "128" (Channel A).
  void begin(uint32_t dout, uint32_t dout2, uint32_t pd_sck, uint8_t gain = 128u, uint32_t sck_mode = OUTPUT);

  // Check if HX711 is ready
  // from the datasheet: When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock
  // input PD_SCK should be low. When DOUT goes to low, it indicates data is ready for retrieval.
  bool is_ready(void);

  // Wait for the HX711 to become ready
  void wait_ready(unsigned long delay_ms = 0);
  bool wait_ready_retry(unsigned int retries = 3u, unsigned long delay_ms = 0);
  bool wait_ready_timeout(unsigned long timeout = 1000u, unsigned long delay_ms = 0);

  // set the gain factor; takes effect only after a call to read()
  // channel A can be set for a 128 or 64 gain; channel B has a fixed 32 gain
  // depending on the parameter, the channel is also set to either A or B
  void set_gain(uint8_t gain = 128u);

  // waits for the chip to be ready and returns a reading
  void read(long* readValues, unsigned long timeout = 1000u);

  // returns an average reading; times = how many times to read
  void read_average(long* readValues, uint8_t times = 10u);

  // returns (read_average() - OFFSET), that is the current value without the tare weight; times = how many readings to do
  void get_value(long* readValues, uint8_t times = 1u);

  // returns get_value() divided by SCALE, that is the raw value divided by a value obtained via calibration
  // times = how many readings to do
  void get_units(float* readValues, uint8_t times = 1u);

  // set the OFFSET value for tare weight; times = how many times to read the tare value
  void tare(uint8_t times = 10u);

  // set the SCALE value; this value is used to convert the raw data to "human readable" data (measure units)
  void set_scale(float scale = 1.f, float scale2 = 1.f);

  // get the current SCALE
  void get_scale(float* scaleValues);

  // set OFFSET, the value that's subtracted from the actual reading (tare weight)
  void set_offset(long offset = 0, long offset2 = 0);

  // get the current OFFSET
  void get_offset(long* offsetValues);

  // get current reads counter
  long get_readCounter(void);

  // puts the chip into power down mode
  void power_down(void);

  // wakes up the chip after power down mode
  void power_up(void);
};
