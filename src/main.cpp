#include <Arduino.h>
#include "MSP.h"
#include "Adafruit_LPS35HW.h"
#include "ms4525do.h"
#include "MS5525DSO.h"
#include <Adafruit_NeoPixel.h>

#define MS5525DSO_CMD_RESET     ((uint8_t)0x1e)
#define MS5525DSO_CMD_BASE_PROM ((uint8_t)0xa0)
#define MS5525DSO_CMD_CONVERT   ((uint8_t)0x40)
// Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);
const uint8_t _Q_coeff[pp_MAXPART][6] =
{
  { 15, 17, 7, 5, 7, 21 }, // pp001DS
  { 14, 16, 8, 6, 7, 22 }, // pp002GS
  { 16, 18, 6, 4, 7, 22 }, // pp002DS
  { 16, 17, 6, 5, 7, 21 }, // pp005GS
  { 17, 19, 5, 3, 7, 22 }, // pp005DS
  { 16, 17, 6, 5, 7, 22 }, // pp015GS
  { 16, 17, 6, 5, 7, 22 }, // pp015AS
  { 17, 19, 5, 3, 7, 22 }, // pp015DS
  { 17, 18, 5, 4, 7, 22 }, // pp030AS
  { 17, 18, 5, 4, 7, 22 }, // pp030GS
  { 18, 21, 4, 1, 7, 22 }, // pp030DS
};

/* These #defines are only valid inside a class instance method */
#define Q1_      (_Q_coeff[0][0])
#define Q2_      (_Q_coeff[0][1])
#define Q3_      (_Q_coeff[0][2])
#define Q4_      (_Q_coeff[0][3])
#define Q5_      (_Q_coeff[0][4])
#define Q6_      (_Q_coeff[0][5])
#define P_SENS_ 14430
#define P_OFF_ 9904
#define TC_SENS_ 3520
#define TC_OFF_ 1949
#define T_REF_ 27271
#define T_SENS_ 8050


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
  Serial.begin(9600);
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