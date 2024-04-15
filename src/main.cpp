#include <Arduino.h>
#include "MSP.h"
#include "Adafruit_LPS35HW.h"
#include "ms4525do.h"
#include "MS5525DSO.h"
#include <Adafruit_NeoPixel.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Adafruit_SHT31.h"

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
#define PRESSURE_HZ 20
#define AIRSPEED_HZ 20
#define TEMP_HZ 10
#define MSP_HZ 3
#define NEOPIXEL_HZ 1
#define INITILIZATION_TRIES 20
#define SHT30

// #define NO_MSP
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);
MSP msp;
Adafruit_SHT31 sht = Adafruit_SHT31();
Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();
OneWire oneWire(3);
DallasTemperature dallasTemp(&oneWire);
DeviceAddress dallasAddr;
msp_get_custom_sensors_t payload;
uint32_t firstTime;
bool sht_connected = false;
bool lps_connected = false;
bool dallas_connected = false;
void convertMS5525(uint32_t pres_in, uint32_t temp_in, float *pres_out, float *temp_out);

void setup() {
  delay(250);
  pixels.begin();
  Serial1.begin(115200);
  pixels.setPixelColor(0,pixels.Color(0,0,255));
  pixels.show();
  #ifndef NO_MSP
  msp.begin(Serial1);
  #endif
  uint32_t startTime = millis();
  while (millis()-startTime < 3000) {
    if (sht.begin()) {
      sht_connected = true;
      break;
    }
    #ifdef NO_MSP
    Serial.println("Error communicating with sht");
    #endif
    if (millis() % 1000 > 150) { //Quick red flash, slow
      pixels.setPixelColor(0,pixels.Color(255,0,0));
    } else {
      pixels.setPixelColor(0,pixels.Color(0,0,0));
    }
    pixels.show();
  }
  startTime = millis();
  while (millis()-startTime < 3000) {
    if (lps35hw.begin_I2C()) {
      lps_connected = true;
      break;
    }
    #ifdef NO_MSP
    Serial.println("Couldn't find LPS35HW chip");
    #endif
    if (millis() % 500 > 150) { //Fast red blink
      pixels.setPixelColor(0,pixels.Color(255,0,0));
    } else {
      pixels.setPixelColor(0,pixels.Color(0,0,0));
    }
    pixels.show();
  }
  lps35hw.setDataRate(LPS35HW_RATE_25_HZ);
  lps35hw.resetPressure(); //Absolute pressure mode
  while (millis()-startTime < 3000) { //Flashing red
    dallasTemp.begin(); // Dallas temp
    dallasTemp.getAddress(dallasAddr, 0);
    if (dallasTemp.isConnected(dallasAddr)) {
      dallas_connected = true;
      break;
    }
    if (millis() % 1000 > 500) { //Even pink blink
      pixels.setPixelColor(0,pixels.Color(255,0,255));
    } else {
      pixels.setPixelColor(0,pixels.Color(0,0,0));
    }
    pixels.show();
  }
  dallasTemp.setWaitForConversion(false);
  dallasTemp.setCheckForConversion(false);
  while(!Serial1) {
    if (millis() % 1000 > 500) { //Slow even blue blink
      pixels.setPixelColor(0,pixels.Color(0,0,255));
    } else {
      pixels.setPixelColor(0,pixels.Color(0,0,0));
    }
    pixels.show();
  }
  payload.timeMs = 0;
  payload.humidity = 1;
  payload.temp_SHT = 2;
  payload.pressure_lps = 3;
  payload.temp_lps = 4;
  payload.temp_ds18b20 = 5;
  payload.differential_pressure_up = 6;
  payload.up_die_temp = 7;
  payload.differential_pressure_forward = 8;
  payload.forward_die_temp = 9;
  payload.differential_pressure_side = 10;
  payload.side_die_temp = 11;
  firstTime = millis();
  pixels.setPixelColor(0,pixels.Color(0,255,0));
  pixels.show();
}

void loop() {
  static uint32_t last_time_sht = millis();
  static uint32_t last_time_lps = millis();
  // static uint32_t last_time_airspeed = millis();
  static uint32_t last_time_msp = millis();
  static uint32_t last_time_neopixel = millis();
  static uint32_t last_time_dallas_temp = millis();
  static bool waitingOnDallasConversion = false;

  uint32_t start_time = millis();
  payload.timeMs = start_time-firstTime;
  if (sht_connected && start_time - last_time_sht > 1000/HUMIDITY_HZ) {
    sht.update();
    payload.humidity = sht.readHumidityPacket();
    payload.temp_SHT = sht.readTemperaturePacket();
  }
  start_time = millis();
  if (lps_connected && start_time - last_time_lps > 1000/PRESSURE_HZ) {
    payload.pressure_lps = lps35hw.readPressureRaw();
    payload.temp_lps = lps35hw.readTemperatureRaw();
    last_time_lps = start_time;
  }
  start_time = millis();
  if (dallas_connected && !waitingOnDallasConversion && start_time - last_time_dallas_temp > 1000/TEMP_HZ) {
    dallasTemp.requestTemperatures();
    waitingOnDallasConversion = true;
    last_time_dallas_temp = millis();
  }
  if (dallas_connected && waitingOnDallasConversion && dallasTemp.isConversionComplete()) {
      payload.temp_ds18b20 = dallasTemp.celsiusToRaw(dallasTemp.getTempC((uint8_t*) dallasAddr));
      waitingOnDallasConversion = false;
  }
  #ifndef NO_MSP
  start_time = millis();
  if (start_time - last_time_msp > 1000/MSP_HZ) {
    msp.command(MSP_GET_CUSTOM_SENSORS, &payload, sizeof(payload));
    last_time_msp = start_time;
  }
  #else
  start_time = millis();
  if (start_time - last_time_msp > 1000/1) {
    int32_t stemp, shum;
    float temp, humidity;
    temp = (payload.temp_SHT * 175.0f) / 65535.0f - 45.0f;
    humidity = (payload.humidity * 100.0f) / 65535.0f;
    Serial.print("Millis: ");
    Serial.print(payload.timeMs);
    Serial.print("\t Humidity: ");
    Serial.print(humidity);
    Serial.print(" SHT Temp: ");
    Serial.print(temp);
    Serial.print('\t');

    // Atmospheric pressure
    if (payload.pressure_lps & 0x800000) {
      payload.pressure_lps = (0xff000000 | payload.pressure_lps);
    }
    Serial.print("Atmospheric pressure: ");
    Serial.print(payload.pressure_lps / 4096.0);
    Serial.print("\t");

    Serial.print("Dallas Temp: ");
    Serial.print(dallasTemp.rawToCelsius(payload.temp_ds18b20));
    Serial.println("");
    last_time_msp = start_time;
  }
  #endif
  start_time = millis();
  static bool neopixel_state = true;
  if (start_time - last_time_neopixel > 1000/NEOPIXEL_HZ) {
    if(neopixel_state){
      pixels.setPixelColor(0,pixels.Color(0,255,0));
    } else {
      pixels.setPixelColor(0,pixels.Color(0,0,0));
    }
    pixels.show();
    neopixel_state = !neopixel_state;
    // static uint16_t color = 65432/2;
    // pixels.rainbow(color);
    // pixels.show();
    // color+=1000; if (color >= 65432) {color = 0;}
    last_time_neopixel = start_time;
  }
}

void convertMS5525(uint32_t pres_in, uint32_t temp_in, float *pres_out, float *temp_out) {
  //MS5525
  float p_ms5525_for, t_ms5525_for;
  // Difference between actual and reference temperature
  int64_t dT = temp_in - ((int64_t)T_REF_ << Q5_);

  // Offset at actual temperature
  int64_t off = ((int64_t)P_OFF_ << Q2_) + ((TC_OFF_ * dT) >> Q4_);

  // Sensitivity at actual temperature
  int64_t sens = ((int64_t)P_SENS_ << Q1_) + ((TC_SENS_ * dT) >> Q3_);

  // Temperature compensated pressure
  int64_t tc_press = (((sens * pres_in) >> 21) - off) >> 15;
  p_ms5525_for = tc_press * 0.0001f;

  t_ms5525_for = (2000 + ((dT * T_SENS_) >> Q6_)) * 0.01f;
  *pres_out = p_ms5525_for;
  *temp_out = t_ms5525_for;
}