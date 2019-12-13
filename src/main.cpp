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


//Structs to track OBC internal data

//This struct tracks the last time in millisecond the subsystem returns no error after a system check poll.
typedef struct {
  uint32_t sensor;
} last_working_t;

last_working_t system_last_working;

//This struct tracks the last error code received from each subsystem. If it is 255 then there is a bus error
typedef struct {
  uint8_t sensor;
} error_t;

error_t system_err;

// Function forward declarations
void checkSubsystemHealth(void);
void restartSubsystem(void);
void sendData(void);

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

/*
* This function will poll each subsystem to check whether they are healthy or not
* where it is healthy if the error number they returned is 0, as in there is no error, and 
* there is no bus error. For all the subsystems that follow the specification
* the error code is the first byte that are returned from the I2C request command
*/
void checkSubsystemHealth(void){
  uint8_t check = 0;
  // Copy paste the codes until the next line comment to check for other subsystems
  check = Wire.requestFrom(ADDR_SENSOR,1);
  if (check != 1)
  {
    system_err.sensor = 255; //If there is a bus error then 
  }
  while (Wire.available())
  {
    system_err.sensor = Wire.read();
  }
  if (system_err.sensor == 0) //If there is no error
  {
      system_last_working.sensor = millis();
  }
  //Copy paste until here
}