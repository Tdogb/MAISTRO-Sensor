#include <Arduino.h>
#include "MSP.h"
#include "Adafruit_LPS35HW.h"
#include "ms4525do.h"
#include <Adafruit_NeoPixel.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_SPIFlash.h>
#include "Adafruit_SHT31.h"
#include "mavlink/MAVLink.h"
#include "wiring_private.h" // pinPeripheral() function
#include "SensorFilter.h"
#include "flash_config.h"
#include <SdFat.h>
#include "AllSensors_DLV.h"


/*
HEADER
ADDR 0x00-0x07: last addr
ADDR 0x08-0x09: CODE VERSION

BODY
START:
*/

enum flash_ids {
  TIME = 0,
  HUMIDITY = 1,
  PRESSURE = 2,
  AIRSPEED = 3,
  TEMP = 4,
  UV = 5
};
#define VERSION 1
#define DEFAULT_STARTING_ADDR 0x50

#define HUMIDITY_HZ 1
#define PRESSURE_HZ 25
#define AIRSPEED_HZ 50
#define TEMP_HZ 10
#define MSP_HZ 20
#define NEOPIXEL_HZ 1
#define UV_HZ 30
#define FLASH_HZ 30

#define MAV_TIMING_UPDATE 4  //Used for timing
#define MAV_HEARTBEAT_HZ 4
#define MAV_RAW_IMU_HZ 4
#define MAV_ATTITUDE_HZ 4
#define MAV_GPS_HZ 4
#define MAV_HUMIDITY_HZ 1
#define MAV_PRESSURE_HZ 4

#define DALLAS_PIN 1
#define MAVLINK_RX_PIN 9 //RX is MISO
#define MAVLINK_TX_PIN 10 //TX is MOSI

#define SD_CS 0
#define MAXPAGESIZE 256

// #define PLANE
//#define FLASH
// #define NO_MSP

void uint32ToUint8Array(uint32_t value, uint8_t* array);
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);
#ifdef FLASH
Adafruit_SPIFlash flash(&flashTransport);
#endif
MSP msp;
Adafruit_SHT31 sht = Adafruit_SHT31();
Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();
OneWire oneWire(DALLAS_PIN);
DallasTemperature dallasTemp(&oneWire);
DeviceAddress dallasAddr;
// bfs::Ms4525do diff_pres_forward;
AllSensors_DLV diff_pres(&Wire, AllSensors_DLV::SensorType::DIFFERENTIAL, 10.0);
msp_get_custom_sensors_t payload;
uint32_t firstTime;
bool sht_connected = false;
bool lps_connected = false;
bool dallas_connected = false;
bool diff_pres_forward_connected = false;
bool uv_connected = false;
uint32_t last_flash_addr = DEFAULT_STARTING_ADDR;
uint32_t blackbox_number = 0;
uint16_t PRatio = 0;
uint8_t sample_rate = 0;
uint64_t mcu_micros_delta = 0;
bool mcu_millis_less_than_msp = false;
// SdFat sd;
// File32 dataFile;
// uint8_t buffer[MAXPAGESIZE], buffer2[MAXPAGESIZE];

uint64_t calculate_micros(void);

SensorFilter<typeof(payload.humidity)> humidity_filter(HUMIDITY_HZ,MSP_HZ,FLASH_HZ);
SensorFilter<typeof(payload.temp_SHT)> temp_sht_filter(HUMIDITY_HZ,MSP_HZ,FLASH_HZ);
SensorFilter<typeof(payload.pressure_lps)> pressure_lps_filter(PRESSURE_HZ,MSP_HZ,FLASH_HZ);
SensorFilter<typeof(payload.temp_lps)> temp_lps_filter(PRESSURE_HZ,MSP_HZ,FLASH_HZ);
SensorFilter<typeof(payload.temp_ds18b20)> temp_ds18b20_filter(TEMP_HZ,MSP_HZ,FLASH_HZ);
SensorFilter<typeof(payload.differential_pressure_forward)> differential_pressure_forward_filter(AIRSPEED_HZ,MSP_HZ,FLASH_HZ);
SensorFilter<typeof(payload.forward_die_temp)> forward_die_temp_filter(AIRSPEED_HZ,MSP_HZ,FLASH_HZ);
SensorFilter<typeof(payload.differential_pressure_up)> uv_filter(UV_HZ,MSP_HZ,FLASH_HZ);

Uart Serial2(&sercom2,MAVLINK_RX_PIN,MAVLINK_TX_PIN,SERCOM_RX_PAD_0,UART_TX_PAD_2);
uint32_t start_time;

void SERCOM2_Handler()
{
  Serial2.IrqHandler();
}

void setup() {
  start_time = millis();
  Wire.setClock(220000);
  // Wire.begin();
  pixels.begin();
  pixels.setBrightness(10);
  pixels.setPixelColor(0,pixels.Color(255,0,0));
  pixels.show();
  delay(200);
  #ifdef FLASH
  flash.begin();
  last_flash_addr = flash.read32(0);
  if (last_flash_addr == 0) { last_flash_addr = DEFAULT_STARTING_ADDR; }
  #endif
  Serial.begin(115200);
  #ifdef PLANE
  // pinPeripheral(MAVLINK_RX_PIN,PIO_SERCOM);
  pinPeripheral(MAVLINK_TX_PIN,PIO_SERCOM);
  pinPeripheral(3,PIO_ANALOG);
  Serial1.begin(115200);
  uv_connected = true;
  #else
  pinPeripheral(MAVLINK_RX_PIN,PIO_SERCOM);
  pinPeripheral(MAVLINK_TX_PIN,PIO_SERCOM);
  Serial1.begin(115200); //FC
  Serial2.begin(4800); //RX
  uv_connected = false;
  #endif

  while (!Serial1) {}

  #ifdef FLASH
  uint32_t start_time = millis();
  while (millis() - start_time < 3000) {
    Serial.println("Type r  to read, or anything else to bypass");
    pixels.setPixelColor(0,pixels.Color(252, 186, 3));
    pixels.show();
    if (Serial.available() >= 1) {
      char in = Serial.read();

      if (in == 'r') {
        Serial.println("Reading");
        // Dump flash
        uint8_t buffer[128];
        flash.readBuffer(0,buffer,sizeof(buffer));
        for (int i = 0; i < sizeof(buffer); i++) {
          Serial.print(buffer[i]);
          Serial.print(" ");
        }
      }
      Serial.println("Type e to exit or E to erase");
      in = Serial.read();
      if (in == 'E') {
        flash.eraseChip();
      } else {
        Serial.println("Exiting");
      }
      break;
    }
    delay(500);
  }
  pixels.setPixelColor(0,pixels.Color(0, 0, 0));
  pixels.show();
  #endif

  #ifndef NO_MSP
  msp.begin(Serial1);
  // msp_get_blackbox_t response;
  start_time = millis();
  #ifndef PLANE
  // while (millis() - start_time < 1000) {
  //   if (msp.request(MSP_BLACKBOX_CONFIG, &response, sizeof(response))) {
  //     blackbox_number = response.blackbox_number_tornado;
  //     sample_rate = response.sample_rate;
  //     PRatio = response.PRatio;
  //     break;
  //   }
  //   delay(100);
  // }

  #endif
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
    lps35hw.setDataRate(LPS35HW_RATE_25_HZ);
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
  temp_ds18b20_filter.disable();

  #ifndef PLANE
  // startTime = millis();
  // diff_pres_forward.Config(&Wire, 0x28, 1.0f, -1.0f);
  // while (millis()-startTime < 1000) {
  //   if (diff_pres_forward.Begin()) {
  //     diff_pres_forward_connected = true;
  //     Serial.println("begun");
  //     break;
  //   }
  //   Serial.println("not begun");
  //   if (millis() % 1000 > 500) { //Even white blink
  //     pixels.setPixelColor(0,pixels.Color(255,255,255));
  //   } else {
  //     pixels.setPixelColor(0,pixels.Color(0,0,0));
  //   }
  //   pixels.show();
  //   delay(50);
  // }
  diff_pres.setTemperatureUnit(AllSensors_DLV::TemperatureUnit::CELCIUS);
  diff_pres.setPressureUnit(AllSensors_DLV::PressureUnit::PSI);
  diff_pres_forward_connected = true;


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
}

void loop() {
  static uint32_t last_time_sht = 0;
  static uint32_t last_time_lps = 0;
  static uint32_t last_time_msp = millis();
  static uint32_t last_time_neopixel = 0;
  static uint32_t last_time_dallas_temp = 0;
  static uint32_t last_time_uv = 0;
  static uint32_t last_time_diff_pres = 0;
  static uint32_t last_time_flash = millis();
  static uint32_t last_time_timing = 0;
  static bool waitingOnDallasConversion = false;
  static mavlink_command_long_t received_msg;
  // static uint64_t mcu_micros_delta = 0;
  start_time = millis();
  payload.timeMs = start_time-firstTime;
  // if (start_time - last_time_timing > 1000/MAV_TIMING_UPDATE) {
  //   msp_sonar_altitude_t packet;
  //   msp.request(MSP_SONAR_ALTITUDE,&packet,sizeof(packet)); //Millis from the fc
  //   payload.timeMs = packet.altitude;
  //   start_time = millis();
  //   mcu_millis_less_than_msp = start_time < packet.altitude;
  //   if (mcu_millis_less_than_msp) {
  //     mcu_micros_delta = 1000*(uint64_t)(packet.altitude - start_time);
  //   } else {
  //     mcu_micros_delta = 1000*(uint64_t)(start_time - packet.altitude);
  //   }
  // }
  start_time = millis();
  if (sht_connected && start_time - last_time_sht > 1000/HUMIDITY_HZ) {
    Serial.println("sht start");
    sht.update();
    Serial.println("sht after update");

    humidity_filter.update_sensor(sht.readHumidityPacket());
    temp_sht_filter.update_sensor(sht.readTemperaturePacket());
    Serial.println("sht end");
    last_time_sht = start_time;
  }
  start_time = millis();
  if (lps_connected && start_time - last_time_lps > 1000/PRESSURE_HZ) {
    Serial.println("lps start");

    pressure_lps_filter.update_sensor(lps35hw.readPressureRaw());
    Serial.println("done pressure lps");
    temp_lps_filter.update_sensor(lps35hw.readTemperatureRaw());
    Serial.println("done temp lps");
    last_time_lps = start_time;
    Serial.println("lps end");

  }
  start_time = millis();
  if (dallas_connected && !waitingOnDallasConversion && start_time - last_time_dallas_temp > 1000/TEMP_HZ) {
    Serial.println("request start");
    dallasTemp.requestTemperatures();
    waitingOnDallasConversion = true;
    last_time_dallas_temp = start_time;
    Serial.println("request done");

  }
  if (dallas_connected && waitingOnDallasConversion && dallasTemp.isConversionComplete()) {
    Serial.println("dallas");
    uint16_t temporary_val = dallasTemp.celsiusToRaw(dallasTemp.getTempC((uint8_t*) dallasAddr));
      temp_ds18b20_filter.update_sensor(temporary_val);
      waitingOnDallasConversion = false;
    Serial.println("dallas done");

  }
  start_time = millis();
  if (diff_pres_forward_connected && start_time - last_time_diff_pres > 1000/AIRSPEED_HZ) {
    Serial.println("diff pres");

    diff_pres.readData();
    differential_pressure_forward_filter.update_sensor((uint32_t)(diff_pres.raw_p));
    forward_die_temp_filter.update_sensor((uint16_t)(diff_pres.temperature * 100));
    last_time_diff_pres = start_time;
    Serial.println("diff pres done");

  }
  // start_time = millis();
  // if (uv_connected && start_time - last_time_uv > 1000/UV_HZ) {
  //    uv_filter.update_sensor(analogRead(A0));
  //    last_time_uv = start_time;
  // }

  #ifdef FLASH
  start_time = millis();
  if (start_time - last_time_flash > 1000/FLASH_HZ) {
    payload.humidity = humidity_filter.output_flash();
    payload.temp_SHT = temp_sht_filter.output_flash();
    payload.pressure_lps = pressure_lps_filter.output_flash();
    payload.temp_lps = temp_lps_filter.output_flash();
    payload.temp_ds18b20 = temp_ds18b20_filter.output_flash();
    payload.differential_pressure_up = uv_filter.output_flash();
    char prefix[] = "Lou";
    last_flash_addr += flash.writeBuffer(last_flash_addr,(uint8_t*)&prefix,sizeof(prefix));
    last_flash_addr += flash.writeBuffer(last_flash_addr,(uint8_t*)&payload,sizeof(payload));
    uint8_t last_flash_addr_uint8[4];
    uint32ToUint8Array(last_flash_addr, last_flash_addr_uint8);
    flash.writeBuffer(0,last_flash_addr_uint8,sizeof(last_flash_addr_uint8));
    last_time_flash = start_time;
  }
  #endif

  start_time = millis();
  if (start_time - last_time_msp > 1000/MSP_HZ) {
    Serial.println("MSP");
    payload.humidity = humidity_filter.output();
    payload.temp_SHT = temp_sht_filter.output();
    payload.pressure_lps = pressure_lps_filter.output();
    payload.temp_lps = temp_lps_filter.output();
    payload.temp_ds18b20 = temp_ds18b20_filter.output();
    payload.differential_pressure_forward = differential_pressure_forward_filter.output();
    payload.forward_die_temp = forward_die_temp_filter.output();
    // payload.differential_pressure_up = uv_filter.output();
    msp.command(MSP_GET_CUSTOM_SENSORS, &payload, sizeof(payload));
    last_time_msp = start_time;
  }

  #ifndef PLANE
  static bool serial2_detected = false;
  static uint32_t last_time_heartbeat_mav = 0, last_time_raw_imu_mav = 0, last_time_raw_gps_mav = 0, last_time_attitude_mav = 0, last_time_humidity_mav = 0, last_time_raw_pressure_mav = 0;
  while (!serial2_detected && !Serial2) {}

  // if (Serial2.available() >= sizeof(mavlink_set_home_position_t) + 25) {
  static mavlink_message_t msg; 
  static mavlink_status_t status;
  while (Serial2.available() > 0) {
  Serial.println("Mav available");
    uint8_t c = Serial2.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
        Serial.println(mavlink_msg_command_int_get_command(&msg));
        if (mavlink_msg_command_int_get_command(&msg) == MAV_CMD_DO_SET_HOME) {
          msp_set_wp_t set_home_payload;
          set_home_payload.lat = mavlink_msg_command_int_get_x(&msg);
          set_home_payload.lon = mavlink_msg_command_int_get_y(&msg);
          Serial.printf("%i %i\n",set_home_payload.lat,set_home_payload.lon);
          set_home_payload.waypointNumber = 0;
          msp.send(MSP_SET_WP, &set_home_payload, sizeof(set_home_payload));
        }
      }
    }
  }
  // }
  serial2_detected = true;
  start_time = millis();
  if (start_time - last_time_heartbeat_mav > 1000/MAV_HEARTBEAT_HZ) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 0, MAV_STATE_STANDBY);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.println("Heartbeat");
    Serial2.write(buf, len);
    last_time_heartbeat_mav = start_time;
  }
  start_time = millis();
  if (start_time - last_time_raw_imu_mav > 1000/MAV_RAW_IMU_HZ) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    msp_raw_imu_t msp_packet;
    msp.request(MSP_RAW_IMU,&msp_packet,sizeof(msp_packet));
    mavlink_msg_raw_imu_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, calculate_micros(), msp_packet.acc[0],msp_packet.acc[1],msp_packet.acc[2]
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
    mavlink_msg_attitude_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, calculate_micros(), msp_packet.roll/10.0f, msp_packet.pitch/10.0f, msp_packet.yaw/1.0f,0.0f,0.0f,0.0f);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
    last_time_attitude_mav = start_time;
  }
  start_time = millis();
  if (start_time - last_time_raw_gps_mav > 1000/MAV_GPS_HZ) {
    Serial.println("gps");   
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    msp_raw_gps_t msp_packet;
    msp.request(MSP_RAW_GPS,&msp_packet,sizeof(msp_packet));
    mavlink_msg_gps_raw_int_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, calculate_micros(), msp_packet.fixType, msp_packet.lat, msp_packet.lon,
    msp_packet.alt,UINT16_MAX,UINT16_MAX,msp_packet.groundSpeed,msp_packet.groundCourse,msp_packet.numSat,0,0,0,0,0,0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
    last_time_raw_gps_mav = start_time;
  }
  start_time = millis();
  if (start_time - last_time_humidity_mav > 1000/MAV_HUMIDITY_HZ) {
    Serial.println("hum");
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_hygrometer_sensor_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, calculate_micros(), temp_sht_filter.output(), humidity_filter.output());
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
    last_time_humidity_mav = start_time;
  }
  start_time = millis();
  if (start_time - last_time_raw_pressure_mav > 1000/MAV_PRESSURE_HZ) {
    Serial.println("pres");
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_raw_pressure_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, calculate_micros(),pressure_lps_filter.output(),differential_pressure_forward_filter.output(),forward_die_temp_filter.output(),temp_lps_filter.output());
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
    last_time_raw_pressure_mav = start_time;
  }
  #endif
  start_time = millis();
  static bool neopixel_state = true;
  if (start_time - last_time_neopixel > 1000/NEOPIXEL_HZ) {
    Serial.println("Neo");
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

uint64_t calculate_micros(void) {
  if (mcu_millis_less_than_msp) {
    return micros() + mcu_micros_delta;
  } else {
    return micros() - mcu_micros_delta;
  }
}

void uint32ToUint8Array(uint32_t value, uint8_t* array) {
  array[0] = (uint8_t)(value & 0xFF);          // Extract the least significant byte
  array[1] = (uint8_t)((value >> 8) & 0xFF);   // Extract the second least significant byte
  array[2] = (uint8_t)((value >> 16) & 0xFF);  // Extract the second most significant byte
  array[3] = (uint8_t)((value >> 24) & 0xFF);  // Extract the most significant byte
}