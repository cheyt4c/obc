#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>


#define PRINT_BUF_SIZE 200
static char printBuf[PRINT_BUF_SIZE];

//############################################################
//Contents common to other subsystems
// * NOTE: be sure to update common debugging contents in this code as well

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

//############################################################
//Structs to track OBC internal data

//This struct tracks the following:
// * lastHealthCheckMillis: last time in ms a system health check call was made.
// * lastReadMillis: last time in ms a successful data read was made.
// 
// * lastWorkingMillis: last time in ms the subsystem returns no 
//      error after a system check poll. (32 bit unsigned = 49.71 days)
// * lastError: last error code received from each subsystem. If it is
//      255 then there is a bus error.
struct system_state_t {
  uint32_t lastHealthCheckMillis;
  uint32_t lastReadMillis;
  
  uint32_t lastWorkingMillis;
  uint8_t lastError;
};


static system_state_t sensor = {0};
static sensor_packet_t sensorData = {0};

//OBC event intervals (ms)
#define SENSOR_HEALTH_EVENT_MS 1000
#define SENSOR_READ_EVENT_MS 3000
#define SENSOR_RESET_EVENT_MS 5000

//############################################################
// Function forward declarations
void eventCheckSubsystemHealth(int addrI2c, system_state_t& state, uint32_t eventIntervalMillis);
void restartSubsystem(void);

uint8_t readI2c(int addrI2c, uint8_t *buff, size_t bytes);
void eventReadSensorModule(void);

void sendData(void);

void printSensorData(char* label, sensor_packet_t& data, uint32_t rcvTime);

//############################################################
void setup() {
  Serial.begin(9600);
  Wire.begin();
}

//############################################################
//Tight-running infinite loop: make events non-blocking if possible
void loop() {
  //Handle read events - poll subsystems for system status
  eventReadSensorModule();

  //Handle reset events - if the subsystem is hanging or if 
  //it returns a error message, after 5 second power cycle the subsystem


  //Handle write events
  
  //poll sensor module for current sensor data every 
  //3 seconds, and then send it to groundstation through UHF comms module


}

/*
* This function will generically poll a subsystem to check whether they are healthy or not.
* It is healthy if the error number returned is 0 (no error), and there is no bus error. 
* For all the subsystems that follow the specification the error code is
* the first byte that are returned from the I2C request command.
* I2C slave details: https://forum.arduino.cc/index.php?topic=362256.0
*/
void eventCheckSubsystemHealth(int addrI2c, system_state_t& state, uint32_t eventIntervalMillis) {
  uint32_t now = millis();
  if ((now - state.lastHealthCheckMillis) < eventIntervalMillis) return;
  state.lastHealthCheckMillis = now;

  uint8_t check = 0;
  // Copy paste the codes until the next line comment to check for other subsystems
  check = Wire.requestFrom(addrI2c,1);
  if (check != 1)
  {
    state.lastError = 255; //If there is a bus error then
    return;
  }
  while (Wire.available())
  {
    state.lastError = Wire.read();
  }
  if (state.lastError == 0) //If there is no error
  {
      state.lastWorkingMillis = millis();
  }
  //Copy paste until here
}

/*
 * This function will do a generic blocking I2C read on a subsystem.
 * Return value: 0 if success, numeric error code otherwise.
 */ 
uint8_t readI2c(int addrI2c, uint8_t *buff, size_t bytes) {
  if (Wire.requestFrom(addrI2c,bytes) != bytes) {
    return 1; // bus error
  }
  if (Wire.readBytes(buff, bytes) != bytes) {
    return 2;
  }
  return 0;
}

void eventReadSensorModule(void) {
  uint32_t now = millis();
  if ((now - sensor.lastReadMillis) < SENSOR_READ_EVENT_MS) return;

  uint8_t readStatus = readI2c(ADDR_SENSOR, reinterpret_cast<uint8_t*>(&sensorData), 
                               sizeof(sensorData));
  if (readStatus != 0) return; // re-read on next event loop

  // check CRC?
  sensor.lastReadMillis = now;
  printSensorData(const_cast<char*>("Sensor"), sensorData, sensor.lastReadMillis);
}


//############################################################
//Debugging contents common to other subsystems

//Packet struct print debug
void printSensorData(char* label, sensor_packet_t& data, uint32_t rcvTime) {
  snprintf(printBuf, PRINT_BUF_SIZE,
          "[%s @ %lu]: %d,%lu,%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",      
          label, rcvTime,
          data.err, data.rtcTime, data.gpsTime, data.millisTime,
          data.humidity, data.temp1, data.temp44, data.temp45, data.temp46, 
          data.pressure, data.lat, data.lng, 
          data.accX, data.accY, data.accZ, data.rotX, data.rotY, data.rotZ);
  Serial.print(printBuf);
}