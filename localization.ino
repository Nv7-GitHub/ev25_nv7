// CONFIGURE THESE
const float distTarget = 7;
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

// Variables for control
float omega = 0;
float vel = 0;

// Vehicle parameters
const float mPerDeg = (1.0f/360.0f) * 2 * PI * 0.0254; // Calculated from wheel diameter
const float trackWidth = 150.0/1000.0; // mm converted to m

void loopLocalization() {
  loopCnt++; // To see loop rate

  float left = readAngleL();
  float right = readAngleR();
  float deltaL = normalizeDelta(prevLeft - left);
  float deltaR = normalizeDelta(right - prevRight);

  // Calculate heading
  float dT = (micros() - prevTime)/1000000.0f;

  // Encoder-based angle calculation
  float dTheta = (deltaR - deltaL)/trackWidth * mPerDeg;
  omega = dTheta/dT;

  // IMU-based angle calculation
  //omega = readGyro();
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
  vel = distAvg/dT;
}

float axialDist() {
  return distAxial;
}

float lateralDist() {
  return distLateral;
}

float angVel() {
  return omega;
}

float linVel() {
  return vel;
}

float targetDist() {
  return distTarget;
}

float heading() {
  return theta;
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

void done() {
  doneTime = millis() - start;
}

float getTime() {
  return ((float)(millis() - start))/1000.0f;
}

void printResults() {
  if (!Serial || (start - doneTime) == 0) {
    LEDWrite(0.05, 0.05, 0.05); // Faint white: turned on
    return;
  }
  LEDWrite(0.0, 0.1, 0.1); // Faint purple: finished with a run

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