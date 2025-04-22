#include <Arduino.h>
#include <Wire.h>
#include <MadgwickAHRS.h> // https://github.com/arduino-libraries/MadgwickAHRS/

// Added for ESP-NOW communication
#include <esp_now.h>
#include <WiFi.h>

#include "TM1637.h"

// Un-comment to replace status codes with a counter
// #define BENCHMARKING_MODE

// active MPU6050 address is 1101001, inactive is 1101000
#define ADDRESS 0x69 // nice
#define PIN_SCL 13 // SCL pin for MPU6050
#define PIN_SDA 14 // SDA pin for MPU6050

#define MPU1 25
#define MPU2 33
#define MPU3 32

#define ACCEL_CONFIG 0x1C     // Accelerometer configuration address
#define GYRO_CONFIG  0x1B     // Gyro configuration address

#define ACCEL_SENSITIVITY 0 // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define GYRO_SENSITIVITY 0 // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

#define LOOP_TIME 10 // time in ms, (10 = 100Hz)
#define MADGWICK_FREQ (1000 / LOOP_TIME)

// a bit hacky but this slows down the printouts
#define PRINTOUT_COOLDOWN 10 // 10 loops per printout

#define RAD2DEG ( 180 / 3.1416 )


// TM1637 stuff
#define PIN_DIO 27 // DIO for TM1637 seven segment display
#define PIN_CLK 26 // CLK for TM1637 seven segment display
#define BLANK 11
#define MINUS 10

// status codes
#define STATUS_OK 0x00
#define STATUS_FINISHED_ALL_SETUP 0x01
#define STATUS_FINISHED_PARTIAL_SETUP 0x02 // sent after each sub-device is setup successfully
#define STATUS_SETUP_ERROR 0x03 // sent if partial setup fails
#define STATUS_STARTING_COMMS_SETUP 0x04 // tbh this shouldn't even be sent???
#define STATUS_STARTING_SSEG_SETUP 0x05
#define STATUS_STARTING_I2C_SETUP 0x06
#define STATUS_STARTING_MPU1_CALIBRATION 0x07
#define STATUS_STARTING_MPU2_CALIBRATION 0x08
#define STATUS_STARTING_MPU3_CALIBRATION 0x09
#define STATUS_WARNING_HIGH_DEVIATION 0x0A // sent when one delta is high
#define STATUS_WARNING_EXTREME_DEVIATION 0x0B // sent when BOTH deltas are high
#define STATUS_CALC_ERROR 0x0C // sent if mpuCalc() fails
#define STATUS_I2C_COMMS_ERROR 0x0D // sent if an I2C device fails to respond
#define STATUS_SSEG_DISPLAY_ERROR 0x0E // sent if an error happens when trying to update the SSEG display
#define STATUS_UNKNOWN_ERROR 0x0F // max value for an error code is 0x0F, the upper 4 bytes are unused

#define DEVIATION_THRESHOLD 5 // number of degrees deviated required to trigger HIGH or EXTREME warnings

long currentMs = 0; // current time im milliseconds, updated each time loop() is run
long previousMs = 0; // last time loop() was run in milliseconds
bool setupFinished = false;

int printoutCounter = 0;

float pitch; // Rotation around X axis
float roll; // Rotation around Y axis
float yaw; // Rotation around Z axis

int totalDeviations; // used to count how many deviations are present in pitch & roll

int yawOffset;

int8_t HIDE_DECIMALS[6] = {0, 0, 0, 0, 0, 0};

int loadingIndex = 0;
char LOADING[20][20] = {
  "=     ",
  "==    ",
  "===   ",
  "====  ",
  " ==== ",
  "  ====",
  "   ===",
  "    ==",
  "     =",
  "     =",
  "     =",
  "    ==",
  "   ===",
  "  ====",
  " ==== ",
  "====  ",
  "===   ",
  "==    ",
  "=     ",
  "=     "
};


void sendPacket(int, int, int);
void sendStatusUpdate(int);
void espNowSetup(void);
void mpuSetup(int);
void mpuCalc(void);
void transmit(char, bool);
void transmit(char, char, bool);
void setActiveMpu(int);
float vectorToAngle(float, float, float);
void sortFloats(float*);
void updateDisplay(int, int);
int countDeviation(float);
void playLoadingAnimation(void);
void printData(void);

// ESP32-C3 receiver address (64:e8:33:84:92:6c)
uint8_t receiverAddress[] = {0x64,0xe8,0x33,0x84,0x92,0x6c};

class XYZ {
  public:
    float x;
    float y;
    float z;
};

class XYZint {
  public:
    int16_t x;
    int16_t y;
    int16_t z;
};

class MpuData {
  public:
    XYZ accel;
    XYZ accelOffset;
    XYZ gyro;
    XYZint gyroRaw;
    XYZint gyroOffset;
    XYZ accelAngle; // X = pitch, Y = roll, no Z
    XYZ gyroAngle; // X = pitch, Y = roll, no Z

    Madgwick madgwickFilter;
};

// packet class to be sent over ESP-NOW
// technically a "struct" since I don't know if classes can be sent
typedef struct rotationPacket {
    char yaw; // -180 -> 180
    char pitch; // -90 -> 90
    char roll; // -90 -> 90
    char status; // see status codes above
} rotationPacket;

MpuData* IMU; // Active MPU, assigned using setActiveMpu()
MpuData mpu1; // MPU6050 unit 1
MpuData mpu2; // MPU6050 unit 2
MpuData mpu3; // MPU6050 unit 3

TM1637 sevenSegment;

float pitches[3] = {0, 0, 0};
float rolls[3] = {0, 0, 0};
float yaws[3] = {0, 0, 0};

rotationPacket packet;
esp_now_peer_info_t espReceiver;
