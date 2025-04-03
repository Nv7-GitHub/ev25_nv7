// CONFIGURE THESE
const float targetDist = 7;
float distAxial = 0; // If it goes too far, increase this
float distLateral = 0; // If it goes left normally, make this positive
float theta = 0;

// Variables for localization & analysis
float prevLeft = 0;
float prevRight = 0;
unsigned long prevTime = 0;
long start = 0;
long doneTime = 0;
long loopCnt = 0;

// Vehicle parameters
const float mPerDeg = (1.0f/360.0f) * 2 * PI * 0.0254;
const float P_forward = 0.8;
const float P_difference = 2.0;
const float D = 0.15;
const float maxSpeed = 1.0;
const float trackWidth = 150.0/1000.0; // mm converted to m

void loopLocalization() {
  loopCnt++; // To see loop rate

  float left = readAngleL();
  float right = readAngleR();
  float deltaL = normalizeDelta(prevLeft - left);
  float deltaR = normalizeDelta(right - prevRight);

  // Calculate heading
  float dTheta = (deltaR - deltaL)/trackWidth * mPerDeg; // Encoder-based angle calculation
  // IMU-based angle calculation
  //float dT = (micros() - prevTime)/1000000.0f;
  //float omega = readGyro();
  //float dTheta = dT * omega;
  prevTime = micros();
  theta += dTheta;

  // Make it wrap-around
  while (theta > PI) {
    theta -= 2*PI;
  }
  while (theta < -PI) {
    theta += 2*PI;
  }

  // Calculate odometry
  float distAvg = (deltaL + deltaR)/2.0f * mPerDeg;
  distAxial += distAvg * cos(theta);
  distLateral += distAvg * sin(theta);
}

float axialDist() {
  return distAxial;
}

float lateralDist() {
  return distLateral;
}

// Handles wrap-around in encoder reading (since its absolute pos)
float normalizeDelta(float delta) {
  if (delta < -180.0f) {
    delta += 360.0;
  } else if (delta > 180.0f) {
    delta -= 360.0;
  }
  return fmod(delta, 180);
}

void resetLocalization() {
  prevLeft = readAngleL();
  prevRight = readAngleR();
  start = millis();
  prevTime = micros();
}

void printResults() {
  if (!Serial || (start - doneTime) == 0) {
    LEDWrite(0.05, 0.05, 0.05);
    return;
  }
  LEDWrite(0.0, 0.1, 0.1);

  Serial.print("Done! axialDist:");
  Serial.print(distAxial);
  Serial.print(",lateralDist:");
  Serial.print(distLateral);
  Serial.print(",time:");
  float time = ((float)doneTime)/1000.0f;
  Serial.print(time);
  Serial.print(",loopRate:");
  Serial.println(((float)loopCnt)/time);
}