#include <Arduino.h>
#include "MSP.h"
#include "Adafruit_LPS35HW.h"
#include "ms4525do.h"
#include "MS5525DSO.h"
#include <Adafruit_NeoPixel.h>
#include <OneWire.h>
#include <DallasTemperature.h>

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
#define AIRSPEED_HZ 50
#define TEMP_HZ 10
#define MSP_HZ 100
#define NEOPIXEL_HZ 20
#define INITILIZATION_TRIES 20
#define SHT30
// #define NO_MSP

#if defined(SHT30)
#include "Adafruit_SHT31.h"
Adafruit_SHT31 sht = Adafruit_SHT31();

#else
#include "Adafruit_SHT4X.h"
Adafruit_SHT4x sht = Adafruit_SHT4x();
#endif

Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);
MSP msp;
Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();
bfs::Ms4525do pres;
MS5525DSO sensor_pres_forward(pp001DS);
MS5525DSO sensor_pres_side(pp001DS);
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
msp_get_custom_sensors_t payload;
uint32_t firstTime;

void convertMS5525(uint32_t pres_in, uint32_t temp_in, float *pres_out, float *temp_out);

void setup() {
  Serial.begin(115200);
  while(!Serial) {};
  #ifndef NO_MSP
  msp.begin(Serial);
  #endif
  pixels.begin();
  if (!sht.begin()) {
    Serial.println("Error communicating with sht");
    while (1) delay(100);
  } else {
    #ifdef SHT40
    sht.setPrecision(shtX_HIGH_PRECISION);
    sht.setHeater(shtX_NO_HEATER);
    #endif
  }
  if (!lps35hw.begin_I2C()) {
    Serial.println("Couldn't find LPS35HW chip");
    while (1);
  } else {
    lps35hw.setDataRate(LPS35HW_RATE_75_HZ);
    lps35hw.resetPressure(); //Absolute pressure mode
  }
  pres.Config(&Wire, 0x28, 1.0f, -1.0f); //0x28
  if (!pres.Begin()) {
    Serial.println("Error communicating with sensor");
    while(1){}
  }
  // sensor_pres_forward.setOSR(MS5525DSO_OSR_4096);
  if (!sensor_pres_forward.begin(I2C_MS5525DSO_ADDR)) { //0x76  
    Serial.println("FATAL: no se pudo inicializar sensor de presión!");
    while (true) delay(100);
  }
  // sensor_pres_side.setOSR(MS5525DSO_OSR_4096);
  if (!sensor_pres_side.begin(I2C_MS5525DSO_ADDR_ALT)) { //0x77 
    Serial.println("FATAL: no se pudo inicializar sensor de presión!");
    while (true) delay(100);
  }
  sensors.begin(); // Dallas temp
  payload.timeMs = 0;
  payload.humidity = 0;
  payload.temp_SHT = 0;
  payload.pressure_lps = 0;
  payload.temp_lps = 0;
  payload.temp_ds18b20 = 0;
  payload.differential_pressure_up = 0;
  payload.up_die_temp = 0;
  payload.differential_pressure_forward = 0;
  payload.forward_die_temp = 0;
  payload.differential_pressure_side = 0;
  payload.side_die_temp = 0;
  firstTime = millis();
}

void loop() {
  static uint32_t last_time_sht = millis();
  static uint32_t last_time_lps = millis();
  static uint32_t last_time_airspeed = millis();
  static uint32_t last_time_msp = millis();
  static uint32_t last_time_neopixel = millis();
  static uint32_t last_time_dallas_temp = millis();

  uint32_t start_time = millis();
  payload.timeMs = start_time-firstTime;
  if (start_time - last_time_sht > 1000/HUMIDITY_HZ) {
    #if defined(SHT40)
    sensors_event_t _humidity, _tempSHT;
    sht.getEvent(&_humidity, &_tempSHT);
    payload.humidity = sht.get_rh_ticks_raw();
    payload.temp_SHT = sht.get_t_ticks_raw();
    last_time_sht = start_time;
    #else
    sht.update();
    payload.humidity = sht.readHumidityPacket();
    payload.temp_SHT = sht.readTemperaturePacket();
    #endif
  }
  start_time = millis();
  if (start_time - last_time_lps > 1000/PRESSURE_HZ) {
    payload.pressure_lps = lps35hw.readPressureRaw();
    payload.temp_lps = lps35hw.readTemperatureRaw();
    last_time_lps = start_time;
  }
  start_time = millis();
  if (start_time - last_time_airspeed > 1000/AIRSPEED_HZ) {
    if (pres.Read()) {
      payload.differential_pressure_up = pres.pres_counts();
      payload.up_die_temp = pres.die_temp_counts();
    }
    double pressure_ms5525_forward, temperature_ms5525_forward;
    if (sensor_pres_forward.readPressureAndTemperature(&pressure_ms5525_forward, &temperature_ms5525_forward)) {
      payload.differential_pressure_forward = sensor_pres_forward.read_pressure_raw();
      payload.forward_die_temp = sensor_pres_forward.read_temperature_raw();
    }
    // Serial.println(pressure_ms5525_forward);
    double pressure_ms5525_side, temperature_ms5525_side;
    if (sensor_pres_side.readPressureAndTemperature(&pressure_ms5525_side, &temperature_ms5525_side)) {
      payload.differential_pressure_side = sensor_pres_side.read_pressure_raw();
      payload.side_die_temp = sensor_pres_side.read_temperature_raw();
    }
    last_time_airspeed = start_time;
  }
  start_time = millis();
  if (start_time - last_time_dallas_temp > 1000/TEMP_HZ) {
    sensors.requestTemperatures();
    payload.temp_ds18b20 = sensors.celsiusToRaw(sensors.getTempCByIndex(0));
    last_time_dallas_temp = millis();
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
    float pres_converted_forward, temp_converted_forward;
    float pres_converted_side, temp_converted_side;
    convertMS5525(payload.differential_pressure_forward, payload.forward_die_temp, &pres_converted_forward, &temp_converted_forward);
    Serial.printf("0x76 differential pressure: %f die temp: %f\t",pres_converted_forward,temp_converted_forward); //we are saving uint32 as uint16
    convertMS5525(payload.differential_pressure_side, payload.side_die_temp, &pres_converted_side, &temp_converted_side);
    Serial.printf("0x77 differential pressure: %f die temp: %f\t",pres_converted_side,temp_converted_side); //we are saving uint32 as uint16
    //Humidity
    int32_t stemp, shum;
    float temp, humidity;
    temp = (payload.temp_SHT * 175.0f) / 65535.0f - 45.0f;
    humidity = (payload.humidity * 100.0f) / 65535.0f;
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print(" SHT Temp: ");
    Serial.print(temp);
    Serial.print('\t');

    // Atmospheric pressure
    if (payload.pressure_lps & 0x800000) {
      payload.pressure_lps = (0xff000000 | payload.pressure_lps);
    }
    Serial.print("Atmospheric pressure: ");
    Serial.println(payload.pressure_lps / 4096.0);
    last_time_msp = start_time;
  }
  #endif
  start_time = millis();
  if (start_time - last_time_neopixel > 1000/NEOPIXEL_HZ) {
    static uint16_t color = 65432/2;
    pixels.rainbow(color);
    pixels.show();
    color+=1000; if (color >= 65432) {color = 0;}
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