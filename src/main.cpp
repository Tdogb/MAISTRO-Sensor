#include <Arduino.h>
#include "MSP.h"
#include "Adafruit_LPS35HW.h"
#include "ms4525do.h"
#include <Adafruit_NeoPixel.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Adafruit_SHT31.h"
#include "mavlink/MAVLink.h"
#include "wiring_private.h" // pinPeripheral() function
#include "SensorFilter.h"

#define HUMIDITY_HZ 1
#define PRESSURE_HZ 75
#define AIRSPEED_HZ 50
#define TEMP_HZ 10
#define MSP_HZ 10
#define NEOPIXEL_HZ 1
#define UV_HZ 30

#define MAV_HEARTBEAT_HZ 4
#define MAV_RAW_IMU_HZ 4
#define MAV_ATTITUDE_HZ 4
#define MAV_GPS_HZ 4
#define MAV_HUMIDITY_HZ 1
#define MAV_PRESSURE_HZ 4

#define DALLAS_PIN 3
#define MAVLINK_RX_PIN 9
#define MAVLINK_TX_PIN 10

#define PLANE

// #define NO_MSP
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);
MSP msp;
Adafruit_SHT31 sht = Adafruit_SHT31();
Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();
OneWire oneWire(DALLAS_PIN);
DallasTemperature dallasTemp(&oneWire);
DeviceAddress dallasAddr;
bfs::Ms4525do diff_pres_forward;
msp_get_custom_sensors_t payload;
uint32_t firstTime;
bool sht_connected = false;
bool lps_connected = false;
bool dallas_connected = false;
bool diff_pres_forward_connected = false;
bool uv_connected = false;

SensorFilter<typeof(payload.humidity)> humidity_filter(HUMIDITY_HZ,MSP_HZ);
SensorFilter<typeof(payload.temp_SHT)> temp_sht_filter(HUMIDITY_HZ,MSP_HZ);
SensorFilter<typeof(payload.pressure_lps)> pressure_lps_filter(PRESSURE_HZ,MSP_HZ);
SensorFilter<typeof(payload.temp_lps)> temp_lps_filter(PRESSURE_HZ,MSP_HZ);
SensorFilter<typeof(payload.temp_ds18b20)> temp_ds18b20_filter(TEMP_HZ,MSP_HZ);
SensorFilter<typeof(payload.differential_pressure_forward)> differential_pressure_forward_filter(AIRSPEED_HZ,MSP_HZ);
SensorFilter<typeof(payload.forward_die_temp)> forward_die_temp_filter(AIRSPEED_HZ,MSP_HZ);
SensorFilter<typeof(payload.differential_pressure_up)> uv_filter(UV_HZ,MSP_HZ);

Uart Serial2(&sercom2,MAVLINK_RX_PIN,MAVLINK_TX_PIN,SERCOM_RX_PAD_1,UART_TX_PAD_2); //TX is MOSI, RX is MISO
void SERCOM2_Handler()
{
  Serial2.IrqHandler();
}

void setup() {
  pixels.begin();
  pixels.setBrightness(10);
  pixels.setPixelColor(0,pixels.Color(255,0,0));
  pixels.show();
  delay(200);
  // pinPeripheral(MAVLINK_RX_PIN,PIO_SERCOM);
  pinPeripheral(MAVLINK_TX_PIN,PIO_SERCOM);
  Serial.begin(115200);
  #ifdef PLANE
  Serial1.begin(115200);
  pinMode(A0,INPUT);
  uv_connected = true;
  #else
  Serial1.begin(9600);
  #endif
  Serial2.begin(4800);
  while (!Serial1) {}
  #ifndef NO_MSP
  msp.begin(Serial1);
  #endif

  uint32_t startTime = millis();
  while (millis()-startTime < 1000) {
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
    delay(50);
  }
  startTime = millis();
  while (millis()-startTime < 1000) {
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
    delay(50);
  }
  if (lps_connected) {
    lps35hw.setDataRate(LPS35HW_RATE_75_HZ);
    lps35hw.resetPressure(); //Absolute pressure mode
  }
  startTime = millis();
  while (millis()-startTime < 1000) { //Flashing red
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

  #ifndef PLANE
  startTime = millis();
  diff_pres_forward.Config(&Wire, 0x28, 1.0f, -1.0f);
  while (millis()-startTime < 1000) {
    if (diff_pres_forward.Begin()) {
      diff_pres_forward_connected = true;
      break;
    }
    if (millis() % 1000 > 500) { //Even white blink
      pixels.setPixelColor(0,pixels.Color(255,255,255));
    } else {
      pixels.setPixelColor(0,pixels.Color(0,0,0));
    }
    pixels.show();
    delay(50);
  }
  #endif
  while(!Serial1) {
    if (millis() % 1000 > 500) { //Slow even blue blink
      pixels.setBrightness(100);
      pixels.setPixelColor(0,pixels.Color(0,0,255));
    } else {
      pixels.setPixelColor(0,pixels.Color(0,0,0));
    }
    pixels.show();
  }
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
  pixels.setPixelColor(0,pixels.Color(0,255,0));
  pixels.show();
  delay(5000);
}

void loop() {
  static uint32_t last_time_sht = 0;
  static uint32_t last_time_lps = 0;
  static uint32_t last_time_msp = millis();
  static uint32_t last_time_neopixel = 0;
  static uint32_t last_time_dallas_temp = 0;
  static uint32_t last_time_uv = 0;
  static uint32_t last_time_diff_pres = 0;
  static bool waitingOnDallasConversion = false;
  static uint32_t start_time = millis();
  payload.timeMs = start_time-firstTime;
  if (sht_connected && start_time - last_time_sht > 1000/HUMIDITY_HZ) {
    sht.update();
    humidity_filter.update_sensor(sht.readHumidityPacket());
    temp_sht_filter.update_sensor(sht.readTemperaturePacket());
  }
  start_time = millis();
  if (lps_connected && start_time - last_time_lps > 1000/PRESSURE_HZ) {
    pressure_lps_filter.update_sensor(lps35hw.readPressureRaw());
    temp_lps_filter.update_sensor(lps35hw.readTemperatureRaw());
    last_time_lps = start_time;
  }
  start_time = millis();
  if (dallas_connected && !waitingOnDallasConversion && start_time - last_time_dallas_temp > 1000/TEMP_HZ) {
    dallasTemp.requestTemperatures();
    waitingOnDallasConversion = true;
    last_time_dallas_temp = millis();
  }
  if (dallas_connected && waitingOnDallasConversion && dallasTemp.isConversionComplete()) {
      temp_ds18b20_filter.update_sensor(dallasTemp.celsiusToRaw(dallasTemp.getTempC((uint8_t*) dallasAddr)));
      waitingOnDallasConversion = false;
  }
  start_time = millis();
  if (diff_pres_forward_connected && start_time - last_time_diff_pres > 1000/AIRSPEED_HZ) {
    diff_pres_forward.Read();
    differential_pressure_forward_filter.update_sensor(diff_pres_forward.pres_counts());
    forward_die_temp_filter.update_sensor(diff_pres_forward.die_temp_counts());
  }
  start_time = millis();
  if (uv_connected && start_time - last_time_uv > 1000/UV_HZ) {
     uv_filter.update_sensor(analogRead(A0));
  }

  #ifdef PLANE
  start_time = millis();
  if (start_time - last_time_msp > 1000/MSP_HZ) {
    payload.humidity = humidity_filter.output();
    payload.temp_SHT = temp_sht_filter.output();
    payload.pressure_lps = pressure_lps_filter.output();
    payload.temp_lps = temp_lps_filter.output();
    payload.temp_ds18b20 = temp_ds18b20_filter.output();
    payload.differential_pressure_up = uv_filter.output();
    msp.command(MSP_GET_CUSTOM_SENSORS, &payload, sizeof(payload));
    last_time_msp = start_time;
  }
  #else
  static bool serial2_detected = false;
  static uint32_t last_time_heartbeat_mav = 0, last_time_raw_imu_mav = 0, last_time_raw_gps_mav = 0, last_time_attitude_mav = 0, last_time_humidity_mav = 0, last_time_raw_pressure_mav = 0;
  while (!serial2_detected && !Serial2) {}
  serial2_detected = true;
  start_time = millis();
  if (start_time - last_time_heartbeat_mav > 1000/MAV_HEARTBEAT_HZ) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 0, MAV_STATE_STANDBY);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
    last_time_heartbeat_mav = start_time;
  }
  start_time = millis();
  if (start_time - last_time_raw_imu_mav > 1000/MAV_RAW_IMU_HZ) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    msp_raw_imu_t msp_packet;
    msp.request(MSP_RAW_IMU,&msp_packet,sizeof(msp_packet));
    mavlink_msg_raw_imu_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, micros(), msp_packet.acc[0],msp_packet.acc[1],msp_packet.acc[2]
    , msp_packet.gyro[0],msp_packet.gyro[1],msp_packet.gyro[2]
    , msp_packet.mag[0],msp_packet.mag[1],msp_packet.mag[2],0,1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
    last_time_raw_imu_mav = start_time;
  }
  start_time = millis();
  if (start_time - last_time_attitude_mav > 1000/MAV_ATTITUDE_HZ) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    msp_attitude_t msp_packet;
    msp.request(MSP_ATTITUDE,&msp_packet,sizeof(msp_packet));
    mavlink_msg_attitude_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, micros(), msp_packet.roll/10.0f, msp_packet.pitch/10.0f, msp_packet.yaw/1.0f,0.0f,0.0f,0.0f);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
    last_time_attitude_mav = start_time;
  }
  start_time = millis();
  if (start_time - last_time_raw_gps_mav > 1000/MAV_GPS_HZ) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    msp_raw_gps_t msp_packet;
    msp.request(MSP_RAW_GPS,&msp_packet,sizeof(msp_packet));
    mavlink_msg_gps_raw_int_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, micros(), msp_packet.fixType, msp_packet.lat, msp_packet.lon,
    msp_packet.alt,UINT16_MAX,UINT16_MAX,msp_packet.groundSpeed,msp_packet.groundCourse,msp_packet.numSat,0,0,0,0,0,0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
    last_time_raw_gps_mav = start_time;
  }
  start_time = millis();
  if (start_time - last_time_humidity_mav > 1000/MAV_HUMIDITY_HZ) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_hygrometer_sensor_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, micros(), temp_sht_filter.output(), humidity_filter.output());
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
    last_time_humidity_mav = start_time;
  }
  start_time = millis();
  if (start_time - last_time_raw_pressure_mav > 1000/MAV_PRESSURE_HZ) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_raw_pressure_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, micros(),pressure_lps_filter.output(),differential_pressure_forward_filter.output(),forward_die_temp_filter.output(),temp_lps_filter.output());
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
    last_time_raw_pressure_mav = start_time;
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
    last_time_neopixel = start_time;
  }
}