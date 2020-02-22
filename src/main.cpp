#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <FastCRC.h>

#define PRINT_BUF_SIZE 200
static char printBuf[PRINT_BUF_SIZE];

//############################################################
//Contents common to other subsystems
// * NOTE: be sure to update common debugging contents in this code as well

//Subsystem I2C addresses
#define ADDR_SENSOR 0x01

//I2C packet struct for subsystems
#define NO_ERROR 0
#define BUS_ERROR 255

//Pin defs for watchdog reset
#define SENSOR_RST_PIN 5

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
    //uint32_t pad_1;
    uint32_t crc32;
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
  // event types
  uint32_t lastHealthCheckMillis;
  uint32_t lastReadMillis;
  uint32_t lastWriteMillis;
  // error tracking (for watchdog)
  uint32_t lastWorkingMillis;
  uint8_t lastError;
};

//OBC event intervals (ms)
static char const *SENSOR_LABEL = "Sensor";
static char const *COMMS_LABEL = "Comms";
#define SENSOR_HEALTH_EVENT_MS 1000
#define SENSOR_READ_EVENT_MS 3000
#define SENSOR_RESET_EVENT_MS 5000

#define COMMS_WRITE_EVENT_MS 10000

//Initialisation (globals)
HardwareSerial &SerialComms = Serial1;
FastCRC32 CRC32;
static system_state_t sensorState = {0};
static sensor_packet_t sensorData = {0};
static system_state_t commsState = {0};

//############################################################
// Function forward declarations
uint8_t readI2c(int addrI2c, uint8_t *buff, size_t bytes);
void updateErrorState(system_state_t& state, uint8_t errorCode);
void eventCheckSubsystemHealth(int addrI2c, system_state_t& state, uint32_t eventIntervalMillis);
void restartSubsystem(system_state_t *system , uint8_t pin, uint32_t timeoutMillis);

size_t getDataSize(sensor_packet_t& packet);
bool isCrcMatch(uint32_t sentCrc, uint8_t *buff, size_t total_bytes);
void eventReadSensorModule(void);

void eventWriteCommsModule(void);

void printHex(uint8_t *buf, size_t len, bool has_newline=true);
void printTag(char const *label, uint32_t rcvTime);
void printSensorData(char const *label, sensor_packet_t& data, uint32_t rcvTime);


void watchdogSetup(void)
{
// do what you want here
}
//############################################################
void setup() {
  Serial.begin(9600);
  SerialComms.begin(9600);
  Wire.begin();

  pinMode(SENSOR_RST_PIN,OUTPUT);
  digitalWrite(SENSOR_RST_PIN,LOW);
  Serial.println("Starting from reset");
  watchdogEnable(1000);
}

//############################################################
//Tight-running infinite loop: make events non-blocking if possible
void loop() {
  //Handle read events - poll subsystems for system status
  eventReadSensorModule();

  //Handle reset events - if the subsystem is hanging or if 
  //it returns a error message, after 5 second power cycle the subsystem
  restartSubsystem(&sensorState ,SENSOR_RST_PIN, 5000);

  watchdogReset();

  //Handle write events
  eventWriteCommsModule();
}

//############################################################
/*
 * This function will do a generic blocking I2C read on a subsystem.
 * Return value: 0 if success, numeric error code otherwise.
 */ 
#define READI2C_NO_ERROR 0
#define READI2C_BUS_ERROR 1 
#define READI2C_READ_ERROR 2
uint8_t readI2c(int addrI2c, uint8_t *buff, size_t bytes) {
  if (Wire.requestFrom(addrI2c,bytes) != bytes) {
    return READI2C_BUS_ERROR;
  }
  if (Wire.readBytes(buff, bytes) != bytes) {
    return READI2C_READ_ERROR;
  }
  return READI2C_NO_ERROR;
}

void updateErrorState(system_state_t& state, uint8_t errorCode) {
  //Serial.println("UPdating");
  //Serial.println(errorCode);
  state.lastError = errorCode;
  if (state.lastError == NO_ERROR) {
    //Serial.println("No error");
    state.lastWorkingMillis = millis();
      //Serial.println(state.lastWorkingMillis);

  }
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
  if ((now - state.lastReadMillis       ) < eventIntervalMillis) return;
  state.lastHealthCheckMillis = now;

  uint8_t check = 0;
  // Copy paste the codes until the next line comment to check for other subsystems
  check = Wire.requestFrom(addrI2c,1);
  if (check != 1)
  {
    state.lastError = BUS_ERROR; //If there is a bus error then
    return;
  }
  while (Wire.available())
  {
    state.lastError = Wire.read();
  }
  if (state.lastError == NO_ERROR) {
    state.lastWorkingMillis = millis();
  }
  //Copy paste until here
}

//############################################################
size_t getDataSize(sensor_packet_t& packet) {
  return reinterpret_cast<uint8_t*>(&(packet.crc32)) - reinterpret_cast<uint8_t*>(&packet);
}

bool isCrcMatch(uint32_t sentCrc, uint8_t *buff, size_t leadingBytes) {
  uint32_t rcvdCrc = CRC32.crc32( buff, leadingBytes );
  return (rcvdCrc == sentCrc);
}

/*
 * This function handles a read event for the sensor module. Any
 * data processing should also be handled here.
 */
void eventReadSensorModule(void) {
  uint32_t now = millis();
  if ((now - sensorState.lastReadMillis) < SENSOR_READ_EVENT_MS) return;

  sensor_packet_t sensorDataLocal;
  uint8_t readStatus = readI2c(ADDR_SENSOR, reinterpret_cast<uint8_t*>(&sensorDataLocal), 
                               sizeof(sensor_packet_t));
  if (readStatus != READI2C_NO_ERROR) {
    sensorState.lastError = BUS_ERROR;
    return; // retry to read on next event loop
  }

  if ( isCrcMatch(sensorDataLocal.crc32, reinterpret_cast<uint8_t*>(&sensorDataLocal), 
                  getDataSize(sensorDataLocal)) ) {
    sensorData = sensorDataLocal; // keep result
    printSensorData(SENSOR_LABEL, sensorData, now);
    sensorState.lastReadMillis = now; // update state
    updateErrorState(sensorState, sensorData.err);

  } else { // data corrupted? retry to read on next event loop
    printTag(SENSOR_LABEL, now);
    Serial.print("CRC failed:\n");
    printSensorData(SENSOR_LABEL, sensorDataLocal, now);
    //printHex(reinterpret_cast<uint8_t*>(&sensorDataLocal), sizeof(sensorDataLocal)); // for use in debugging
    // TODO - internally log a CRC error code?
  }
}


//############################################################
void eventWriteCommsModule(void) {
  uint32_t now = millis();
  if ((now - commsState.lastWriteMillis) < COMMS_WRITE_EVENT_MS) return;
  //send sensor module pkt
  uint8_t *pktPtr = reinterpret_cast<uint8_t*>(&sensorData);
  size_t pktSize = sizeof(sensor_packet_t);
  SerialComms.write(pktPtr, pktSize);
  snprintf(printBuf, PRINT_BUF_SIZE, "Wrote %lu bytes to %s\n", pktSize, COMMS_LABEL);
  Serial.print(printBuf);
  commsState.lastWriteMillis = now;
}

//############################################################
//Debugging contents common to other subsystems

void printHex(uint8_t *buf, size_t len, bool has_newline) {
  for (size_t i = 0; i < len; i++) {
    if (buf[i] < 16) {Serial.print("0");}
    Serial.print(buf[i], HEX);
  }
  if (has_newline) Serial.print("\n");
}

void printTag(char const *label, uint32_t rcvTime) {
  snprintf(printBuf, PRINT_BUF_SIZE, "[%s,%lu]: ", label, rcvTime);
  Serial.print(printBuf);
}

//Packet struct print debug
void printSensorData(char const *label, sensor_packet_t& data, uint32_t rcvTime) {
  printTag(label, rcvTime);
  snprintf(printBuf, PRINT_BUF_SIZE,
          "%d,%lu,%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
          data.err, data.rtcTime, data.gpsTime, data.millisTime,
          data.humidity, data.temp1, data.temp44, data.temp45, data.temp46, 
          data.pressure, data.lat, data.lng, 
          data.accX, data.accY, data.accZ, data.rotX, data.rotY, data.rotZ);
  Serial.print(printBuf);
}

void restartSubsystem(system_state_t *system, uint8_t pin, uint32_t timeoutMillis){
  // Restart subsystem if it above the timeoutMillis value
  if ((millis() - system->lastWorkingMillis) > timeoutMillis)
  {
    Serial.println("Reseting system");
    digitalWrite(pin,HIGH);
    delay(10);
    digitalWrite(pin,LOW);
    system->lastWorkingMillis = millis(); //This assumes that the system recovers after restart
  }
}
