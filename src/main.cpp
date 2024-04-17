#include <Arduino.h>
#include "MSP.h"
#include "Adafruit_LPS35HW.h"
#include "ms4525do.h"
#include "MS5525DSO.h"
#include <Adafruit_NeoPixel.h>

#define HUMIDITY_HZ 1
#define PRESSURE_HZ 75
#define AIRSPEED_HZ 100
#define MSP_HZ 100
#define NEOPIXEL_HZ 100
#define INITILIZATION_TRIES 20
#define SHT30
// #define NO_MSP

MSP msp;
msp_get_custom_sensors_t payload;
uint32_t firstTime;


void setup() {
  Serial.begin(115200);
  while(!Serial) {};
  #ifndef NO_MSP
  msp.begin(Serial);
  #endif
  payload.timeMs = 3;
  payload.humidity = 8;
  payload.temp_SHT = 5;
  payload.pressure_lps = 7;
  payload.temp_lps = 6;
  payload.temp_ds18b20 = 5;
  payload.differential_pressure_up = 1;
  payload.up_die_temp = 2;
  payload.differential_pressure_forward = 30;
  payload.forward_die_temp = 4;
  payload.differential_pressure_side = 9;
  payload.side_die_temp = 10;
  firstTime = millis();
}

void loop() {
  static uint32_t last_time_msp = millis();
  static uint32_t last_time_neopixel = millis();

  uint32_t start_time = millis();
  payload.timeMs = start_time-firstTime;
  start_time = millis();
  if (start_time - last_time_msp > 1000/MSP_HZ) {
    msp.command(MSP_GET_CUSTOM_SENSORS, &payload, sizeof(payload));
    last_time_msp = start_time;
  }
}