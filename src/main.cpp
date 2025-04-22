#include "main.h"

void setup() {
  try {
    Serial.begin(115200);

    espNowSetup();
    sendStatusUpdate(STATUS_STARTING_COMMS_SETUP);
    sendStatusUpdate(STATUS_FINISHED_PARTIAL_SETUP); // lol

    // setup seven segment display
    try {
      sendStatusUpdate(STATUS_STARTING_SSEG_SETUP);
      sevenSegment.begin(PIN_CLK, PIN_DIO, 6);
      sevenSegment.setBrightness(4);
      sendStatusUpdate(STATUS_FINISHED_PARTIAL_SETUP);
    } catch (...) {
      throw STATUS_SETUP_ERROR;
    }

    // init I2C signal with MPU6050
    try {
      sendStatusUpdate(STATUS_STARTING_I2C_SETUP);
      Wire.setPins(PIN_SDA, PIN_SCL);
      bool success = Wire.begin();
      if (success) {
        sendStatusUpdate(STATUS_FINISHED_PARTIAL_SETUP);
      } else {
        sendStatusUpdate(STATUS_SETUP_ERROR);
        return;
      }
    } catch (...) {
      throw STATUS_SETUP_ERROR;
    }

    // these should need status updates? I hope?
    pinMode(MPU1, OUTPUT); // mpu 1 (main)
    pinMode(MPU2, OUTPUT); // mpu 2
    pinMode(MPU3, OUTPUT); // mpu 3

    Serial.println("Ensure the MPU is held still during calibration...");
    for (int i = 1; i <= 3; i++) {
      try {
        sendStatusUpdate( // send an update about our setup progress
          (i == 1) ? STATUS_STARTING_MPU1_CALIBRATION :
          (i == 2) ? STATUS_STARTING_MPU2_CALIBRATION :
          (i == 3) ? STATUS_STARTING_MPU3_CALIBRATION :
          STATUS_UNKNOWN_ERROR
        );
        mpuSetup(i);
        sendStatusUpdate(STATUS_FINISHED_PARTIAL_SETUP);
        delay(50);
      } catch (...) {
        throw STATUS_SETUP_ERROR; // handled by outer catch
      }
    }

    sendStatusUpdate(STATUS_FINISHED_ALL_SETUP);
    delay(50);
    setupFinished = true;
    yawOffset = -999;
  } catch (char errorCode) {
    sendStatusUpdate(errorCode);
    setupFinished = false;
  }
}

void loop() {
  if (!setupFinished) {
    return;
  }

  currentMs = millis();
  if (currentMs - previousMs < LOOP_TIME) {
    return;
  }

  try {
    for (int i = 1; i <= 3; i++) {
      setActiveMpu(i);
      mpuCalc();
      try {
        pitches[i-1] = IMU->madgwickFilter.getRoll(); // not sure why, but pitch and roll are swapped
        rolls[i-1] = IMU->madgwickFilter.getPitch();
        yaws[i-1] = IMU->madgwickFilter.getYaw();

        pitches[i-1] = -pitches[i-1] + 90; // normalizes pitch between -90 (down) and +90 (up), 0 = level
        if (isnan(pitches[i-1]) || isnan(rolls[i-1]) || isnan(yaws[i-1])) throw STATUS_CALC_ERROR;
      } catch (...) {
        throw STATUS_CALC_ERROR;
      }
    }
  } catch (char errorCode) {
    sendStatusUpdate(errorCode);
  }

  // sorts all rotations, so min = 0, mid = 1, max = 2
  sortFloats(&pitches[0]);
  sortFloats(&rolls[0]);
  sortFloats(&yaws[0]);

  // idk why this fails outside of sendPacket() but it does :(
  // totalDeviations = countDeviation(pitches) + countDeviation(rolls); // yaw deviations not considered

  sendPacket(pitches[1], rolls[1], (yaws[0] + yaws[1] + yaws[2]) / 3);

  printoutCounter++;
  if (printoutCounter % PRINTOUT_COOLDOWN == 0) {
    printData();
  }

  try {
    updateDisplay(pitches[1], rolls[1]);
  } catch (...) {
    sendStatusUpdate(STATUS_SSEG_DISPLAY_ERROR);
  }

  previousMs = currentMs;
}

// Returns the number of HIGH deviations (see DEVIATION_THRESHOLD)
int countDeviation(float* values) {
  char deviationCount = 0;
  for (int i = 0; i < 2; i++) {
    if (abs(values[i] - values[i+1]) > DEVIATION_THRESHOLD) {
      deviationCount++;
    }
  }
  return deviationCount;
}

#ifdef BENCHMARKING_MODE
unsigned char packetCounter = 0;
#endif

void sendPacket(int pitch, int roll, int yaw) {
  if (yawOffset == -999) {
    yawOffset = yaw;
  }
  yaw -= yawOffset;

  char status = STATUS_OK;
  totalDeviations = countDeviation(pitches);// + countDeviation(rolls); // yaw deviations not considered
  if (totalDeviations >= 2) {
    status = STATUS_WARNING_EXTREME_DEVIATION;
  } else if (totalDeviations == 1) {
    status = STATUS_WARNING_HIGH_DEVIATION;
  } else {
    // print status OK if necessary
  }

  #ifdef BENCHMARKING_MODE
    status = packetCounter;
    packetCounter = (packetCounter + 1) % 128;
  #endif

  if (yaw < 0) status = status | 0x80; // use MSB of status byte as negative bit for yaw

  // prep packet to be sent
  packet.status = status;
  packet.pitch = pitch; // -90 -> 90
  packet.roll = roll; // -90 -> 90
  packet.yaw = abs(yaw); // -180 -> 180, negative bit is included as MSB of status byte
  // send packet over ESP-NOW
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t*)&packet, sizeof(packet));
  if (result == ESP_OK) {
    // Serial.println("Rotation packet sent!");
  }
}

void sendStatusUpdate(int status) {
  packet.status = (char)status;
  packet.pitch = 0;
  packet.roll = 0;
  packet.yaw = 0;
  // send packet over ESP-NOW
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t*)&packet, sizeof(packet));
  if (result == ESP_OK) {
    Serial.print("Status packet sent:");
    Serial.println(status);
  }
}

// ESP-NOW connection setup
void espNowSetup() {
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station

  if (esp_now_init() != ESP_OK) {// Init ESP-NOW
      Serial.println("Error initializing ESP-NOW");
      // sendStatusUpdate(-1); // this literally WOULD NOT send???
      return;
  }

  memcpy(espReceiver.peer_addr, receiverAddress, 6); // Register peer
  espReceiver.channel = 0;
  espReceiver.encrypt = false;

  if (esp_now_add_peer(&espReceiver) != ESP_OK) {
    Serial.println("Failed to add receiver, make sure it's turned on!");
    sendStatusUpdate(-1);
  } else {
    Serial.println("ESP-NOW setup completed successfully!");
    sendStatusUpdate(0);
  }
}

void mpuSetup(int mpuNum) {
  if (mpuNum < 1 || mpuNum > 3) {
    Serial.println("Invalid MPU number");
  } else {
    setActiveMpu(mpuNum);
  }
  Serial.print("Starting setup for MPU_");
  Serial.println(mpuNum);

  Wire.begin();
  delay(100);  
  transmit(PWR_MGMT_1, 0x00, true);
  transmit(ACCEL_CONFIG, ACCEL_SENSITIVITY << 3, true); // Specifying output scaling of accelerometer
  transmit(GYRO_CONFIG, GYRO_SENSITIVITY << 3, true); // Specifying output scaling of gyroscope
  delay(100);

  Serial.println("Starting Madgwick filter sampler for this MPU...");
  IMU->madgwickFilter.begin(MADGWICK_FREQ);
  delay(100);

  // Gyroscope calibration
  for (int i = 0; i < 1024; i++) {
    mpuCalc();
    IMU->gyroOffset.z += IMU->gyroRaw.z;
    IMU->gyroOffset.y += IMU->gyroRaw.y;
    IMU->gyroOffset.x += IMU->gyroRaw.x;
    playLoadingAnimation();
    delay(3);
  }
  IMU->gyroOffset.z = (IMU->gyroOffset.z >> 10); // GyZ_offset = GyZ_offset_sum >> 10;
  IMU->gyroOffset.y = (IMU->gyroOffset.y >> 10); // GyY_offset = GyY_offset_sum >> 10;
  IMU->gyroOffset.x = (IMU->gyroOffset.x >> 10); // GyX_offset = GyX_offset_sum >> 10;
  Serial.print("GyroZ offset value = "); Serial.println(IMU->gyroOffset.z);
  Serial.print("GyroY offset value = "); Serial.println(IMU->gyroOffset.y);
  Serial.print("GyroX offset value = "); Serial.println(IMU->gyroOffset.x);

  // Accelerometer calibration
  float xSum = 0;
  float ySum = 0;
  float zSum = 0;
  int xOffset = round(IMU->accel.x);
  int yOffset = round(IMU->accel.y);
  int zOffset = round(IMU->accel.z);
  for (int i = 0; i < 1024; i++) {
    mpuCalc();
    xSum += (IMU->accel.x - xOffset);
    ySum += (IMU->accel.y - yOffset);
    zSum += (IMU->accel.z - zOffset);
    playLoadingAnimation();
    delay(3);
  }
  IMU->accelOffset.x = (xSum / 1024);
  IMU->accelOffset.y = (ySum / 1024);
  IMU->accelOffset.z = (zSum / 1024);
  Serial.print("AccelX offset value = "); Serial.println(IMU->accelOffset.x);
  Serial.print("AccelY offset value = "); Serial.println(IMU->accelOffset.y);
  Serial.print("AccelZ offset value = "); Serial.println(IMU->accelOffset.z);

  Serial.print("Finished setup for MPU_");
  Serial.println(mpuNum);
}

// Runs all necessary IMU calculations on the active IMU
void mpuCalc() {
  try {
    // Request gyroscope data
    transmit(0x43, false); // 0x43 is the address of the first gyro register to read (x)
    Wire.requestFrom(ADDRESS, 6, true);  
    IMU->gyroRaw.x = Wire.read() << 8 | Wire.read(); // 0x43, 0x44 (GYRO_XOUT_H, GYRO_XOUT_L)
    IMU->gyroRaw.y = Wire.read() << 8 | Wire.read(); // 0x45, 0x46 (GYRO_YOUT_H, GYRO_YOUT_L)
    IMU->gyroRaw.z = Wire.read() << 8 | Wire.read(); // 0x47, 0x48 (GYRO_ZOUT_H, GYRO_ZOUT_L)

    // Request accelerometer data
    transmit(0x3B, false);
    Wire.requestFrom(ADDRESS, 6, true); 
    IMU->accel.x = (Wire.read() << 8 | Wire.read()) / 16384.0 - IMU->accelOffset.x; // 0x3B, 0x3C (ACCEL_XOUT_H, ACCEL_XOUT_L)
    IMU->accel.y = (Wire.read() << 8 | Wire.read()) / 16384.0 - IMU->accelOffset.y; // 0x3D, 0x3E (ACCEL_YOUT_H, ACCEL_YOUT_L)
    IMU->accel.z = (Wire.read() << 8 | Wire.read()) / 16384.0 - IMU->accelOffset.z; // 0x3F, 0x40 (ACCEL_ZOUT_H, ACCEL_ZOUT_L)
  } catch (...) {
    throw STATUS_I2C_COMMS_ERROR;
  }

  try {
    // fix accelerometer negatives
    IMU->accel.x -= (IMU->accel.x > 2) ? 4 : 0;
    IMU->accel.y -= (IMU->accel.y > 2) ? 4 : 0;
    IMU->accel.z -= (IMU->accel.z > 2) ? 4 : 0;

    // add mpu6050 offset values
    IMU->gyroRaw.x -= IMU->gyroOffset.x;
    IMU->gyroRaw.y -= IMU->gyroOffset.y;
    IMU->gyroRaw.z -= IMU->gyroOffset.z;

    IMU->gyro.x = IMU->gyroRaw.x / 131;
    IMU->gyro.y = IMU->gyroRaw.y / 131;
    IMU->gyro.z = IMU->gyroRaw.z / 131;

    // gyro-based yaw pitch & roll
    // IMU->gyroAngle.x += IMU->gyroRaw.x * LOOP_TIME / 1000 / 65.536;
    // IMU->gyroAngle.y += IMU->gyroRaw.y * LOOP_TIME / 1000 / 65.536;
    // IMU->gyroAngle.z += IMU->gyroRaw.z * LOOP_TIME / 1000 / 65.536;

    // accelerometer-based pitch & roll
    // IMU->accelAngle.x = vectorToAngle(IMU->accel.x, IMU->accel.y, IMU->accel.z);
    // IMU->accelAngle.y = vectorToAngle(IMU->accel.y, IMU->accel.x, IMU->accel.z);

    IMU->madgwickFilter.updateIMU(
      IMU->gyro.x,  IMU->gyro.y,  IMU->gyro.z, // gyro must be in deg/sec
      IMU->accel.x, IMU->accel.y, IMU->accel.z // accel must be in
    );
  } catch (...) {
    throw STATUS_CALC_ERROR;
  }
}

void updateDisplay(int pitch, int roll) {
  char pitchString[20];
  char rollString[20];
  char displayString[20];

  if (pitch < 0) { // negative
    if (abs(pitch) >= 10) {
      sprintf(pitchString, "%i", pitch);
    } else {
      sprintf(pitchString, " %i", pitch);
    }
  } else { // positive
    if (abs(pitch) >= 10) {
      sprintf(pitchString, " %i", pitch);
    } else {
      sprintf(pitchString, "  %i", pitch);
    }
  }

  if (roll < 0) { // negative
    if (abs(roll) >= 10) {
      sprintf(rollString, "%i", roll);
    } else {
      sprintf(rollString, " %i", roll);
    }
  } else { // positive
    if (abs(roll) >= 10) {
      sprintf(rollString, " %i", roll);
    } else {
      sprintf(rollString, "  %i", roll);
    }
  }

  sprintf(displayString, "%s%s", pitchString, rollString);
  sevenSegment.displayPChar(displayString);
}

// Used when running accelerometer-based angle calculations. Value is returned in degrees.
float vectorToAngle(float a, float b, float c) {
  return atan(a / sqrt( b*b + c*c )) * RAD2DEG;
  // RAD2DEG is necessary since atan returns a radian (all my homies hate radians).
}

// Transmits the given data to the active IMU
void transmit(char address, char value, bool sendStop) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(sendStop);
}

// Transmits the given data to the active IMU
void transmit(char data, bool sendStop) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(data);
  Wire.endTransmission(sendStop);
}

// updates active MPU ("IMU") and performs the necessary hardware outputs
void setActiveMpu(int mpuNum) {
  digitalWrite(MPU1, (mpuNum == 1) ? HIGH : LOW);
  digitalWrite(MPU2, (mpuNum == 2) ? HIGH : LOW);
  digitalWrite(MPU3, (mpuNum == 3) ? HIGH : LOW);
  IMU = (
    mpuNum == 1 ? &mpu1 :
    mpuNum == 2 ? &mpu2 :
    mpuNum == 3 ? &mpu3 :
    NULL // invalid MPU number
  );
  if (IMU == NULL) {
    throw STATUS_UNKNOWN_ERROR;
  }
}

// Used to sort rotations, uses compare-and-swap method
void sortFloats(float* arr) {
    for (int i = 0; i < 2; i++) { // Only need to iterate 2 times (akin to comparing i to i+1)
        for (int j = 0; j < 2 - i; j++) {
            if (arr[j] > arr[j + 1]) { // Swap the elements
                float temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}


bool headerPrinted = false;
void printData() {

  if (!headerPrinted) {
    // Serial.println("time,pitch 1,pitch 2,pitch 3,median,error count");
    Serial.println("time,roll 1,roll 2,roll 3,median,error count");
    headerPrinted = true;
  }

  /* * * * * TIME PRINTOUT * * * * */
  // Serial.print("[");
  // Serial.print(currentMs - previousMs);
  // Serial.print(" ms] ");

  Serial.print("");
  Serial.print((currentMs - previousMs) * printoutCounter / 1000.0);
  Serial.print(",");

  /* * * * * INDIVIDUAL PITCHES PRINTOUT * * * * */
  for (int i = 1; i <= 3; i++) {
    setActiveMpu(i);
    // Serial.print(-IMU->madgwickFilter.getRoll() + 90); // prints pitch
    Serial.print(IMU->madgwickFilter.getPitch()); // prints roll
    Serial.print(",");
  }

  /* * * * * MEDIAN ROTATIONS PRINTOUTS * * * * */
  // Serial.print(pitches[1]);
  // Serial.print(","); 
  Serial.print(rolls[1]);
  // Serial.print(",");
  // Serial.print(yaws[1]);
  

  /* * * * * ROTATION DIFFS PRINTOUTS * * * * */
  // Serial.print("\t");
  // Serial.print(pitches[0] - pitches[1]);
  // Serial.print("\t/\t");
  // Serial.print(pitches[2] - pitches[1]);
  // Serial.print("\t\t\t");
  // Serial.print(rolls[0] - rolls[1]);
  // Serial.print("\t/\t");
  // Serial.print(rolls[2] - rolls[1]);
  // Serial.print("\t\t\t");
  // Serial.print(yaws[0] - yaws[1]);
  // Serial.print("\t/\t");
  // Serial.println(yaws[2] - yaws[1]);


  /* * * * * ACCELEROMETER ESTIMATION PRINTOUTS * * * * */
  // Serial.print(IMU->accelAngle.x);
  // Serial.print("\t");
  // Serial.println(IMU->accelAngle.y);


  /* * * * * GYROSCOPE ESTIMATION PRINTOUTS * * * * */
  // Serial.print(IMU->gyroAngle.x);
  // Serial.print("\t");
  // Serial.print(IMU->gyroAngle.y);
  // Serial.print("\t");
  // Serial.println(IMU->gyroAngle.z);


  /* * * * * DEVIATION STATUS PRINTOUTS * * * * */
  Serial.print(",");
  Serial.print(totalDeviations);


  Serial.println("");
}

void playLoadingAnimation() {
  sevenSegment.displayPChar(LOADING[loadingIndex / 8]);
  loadingIndex = (loadingIndex + 1) % 160;
}
