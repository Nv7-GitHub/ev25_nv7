#include <ICM20689.h>
#include <MT6701.h>
#include <SPI.h>

#define CS_L 13
#define CS_R 20
#define CS_IMU 17
#define BATTERY A0

const float bias = 0.1254;
const int samples = 14000;

ICM20689 imu(SPI, CS_IMU);
MT6701 encL;
MT6701 encR;

void displayError(String error) {
  while (1) {
    Serial.println(error);
    delay(250);
    LEDWrite(1, 0, 0);
    delay(250);
    LEDWrite(0, 0, 0);
  }
}

void setupSensing() {
  SPI.begin();
  SPI1.begin();

  // IMU
  int status = imu.begin();
  if (status < 0) {
    displayError("IMU INITIALIZATION FAILED");
  }
  status = imu.setGyroRange(ICM20689::GYRO_RANGE_250DPS);
  if (status < 0) {
    displayError("IMU GYRO INITIALIZATION FAILED");
  }
  status = imu.setDlpfBandwidth(ICM20689::DLPF_BANDWIDTH_99HZ); // 92 Hz gyro bandwidth
  if (status < 0) {
    displayError("IMU DLPF INITIALIZATION FAILED");
  }

  // Calculate bias (make sure to set bias to 0 before doing this)
  /*float biasAv = 0;
  for (int i = 0; i < samples; i++) {
    biasAv += readGyro();
    LEDWrite(1, 1, 1);
    delayMicroseconds(127); // Gets 8khz, roughly what it gets while running
  }
  biasAv /= (float)samples;
  while (1) {
    Serial.print("bias: ");
    LEDWrite(0, 0, 0);
    Serial.println(biasAv, 4);
  }*/

  // Encoders
  if (!encL.initializeSSI(CS_L)) {
    displayError("LEFT ENCODER INITIALIZATION FAILED");
  }
  if (!encR.initializeSSI(CS_R)) {
    displayError("LEFT ENCODER INITIALIZATION FAILED");
  }

  // Analog
  pinMode(BATTERY, INPUT);
  analogReadResolution(12);
}

// Returns angular velocity on the yaw axis in rad/s
double readGyro() {
  imu.readSensor();
  return -(imu.getGyroZ_rads() * PI/2) - bias; // TODO: Figure out why I need to scale by pi/2
}

// Encoder funcs TODO: Why do I need to do -180 * 2
double readAngleL() {
  return (encL.angleRead()-180.0)*2.0;
}
double readAngleR() {
  return (encR.angleRead()-180.0)*2.0;
}

double batteryVoltage() {
  int val = analogRead(BATTERY);
  double vout = (double)val/4096.0 * 3.3;
  // R1: 3kohm, R2: 1kohm
  return vout*(3000.0 + 1000.0)/1000.0;
}