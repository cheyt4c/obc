#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>

//Subsystem I2C addresses
#define ADDR_SENSOR 0x01

//I2C packet struct for subsystems
typedef struct {
    uint8_t err;
    uint8_t pad[3];
    uint32_t rtcTime;
    uint32_t gpsTime;
    uint32_t millisTime;
    float humidity;
    float temp1;
    float temp44;
    float temp45;
    float temp46;
    float pressure;
    double lat;
    double lng;
    float accX;
    float accY;
    float accZ;
    float rotX;
    float rotY;
    float rotZ;
    uint32_t pad_1;
    //uint32_t crc32;
} sensor_packet_t;


//Struct to track OBC internal data

//This struct tracks the last time in millisecond the subsystem returns no error after a system check poll.
typedef struct {
  uint32_t sensor;
} last_working_t;

void setup() {
  Serial.begin(9600);
  Wire.begin();
}

void loop() {

  //Poll subsystems for system status


  //If the subsystem is hanging or if it returns a error message, after 5 second power cycle the subsystem


  //Poll sensor module for current sensor data every 3 seconds, and then send it to groundstation through UHF comms module

  delay(1);
}