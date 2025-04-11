// CONFIGURE THESE
const float distTarget = 10 - 0.034; // 0.034 is distance from MP to center
float theta = 0;
const float initialDistAxial = 0.01; // If it goes too far, increase this
const float initialDistLateral = -0.115; // If it goes left normally, make this positive

/* Calibration
// ALWAYS TAKE THE ERROR, DIVIDE BY TWO, AND ROUND DOWN

7m: 
initialDistAxial: -0.01
initialDistLateral: -0.132

8.5m:
initialDistAxial: -0.00
initialDistLateral: -0.147

10m:
initialDistAxial: 0.01
initialDistLateral: -0.115
*/

// Variables for localization & analysis
float distAxial = 0; 
float distLateral = 0; 

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
  //float dTheta = (deltaR - deltaL)/trackWidth * mPerDeg;
  //omega = dTheta/dT;
  /*if (omega > 0.3) {
    Serial.print("dT:");
    Serial.print(dT, 10);
    Serial.print(",dL:");
    Serial.print(deltaL, 5);
    Serial.print(",dR:");
    Serial.print(deltaR, 5);
    Serial.print(",dTheta:");
    Serial.print(dTheta, 10);
    Serial.print(",omega:");
    Serial.println(omega);
  }*/

  // IMU-based angle calculation
  omega = readGyro();
  float dTheta = dT * omega;

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

  // Fix deltas
  prevLeft = left;
  prevRight = right;
}

float axialDist() {
  return distAxial;
}

float lateralDist() {
  // Apply initialDistLateral over the course of the first 25% of the run, to avoid slipping due to a sharp turn
  return min((distAxial/distTarget)*4.0f, 1)*initialDistLateral + distLateral;
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
  start = millis();
  prevTime = micros();
  theta = 0;
  distAxial = initialDistAxial;
  distLateral = 0;
  loopCnt = 0;
  for (int i = 0; i < 5; i++) {
    prevLeft = readAngleL();
    prevRight = readAngleR();
    delay(1);
  }
}

void done() {
  doneTime = millis() - start;
}

float getTime() {
  return ((float)(millis() - start))/1000.0f;
}

void printResults() {
  if ((start - doneTime) == 0) {
    LEDWrite(0.05, 0.05, 0.05); // Faint white: turned on
    if (!Serial) {
      return;
    }
    Serial.print("Battery Voltage: ");
    Serial.print(batteryVoltage());
    Serial.println("V");
    return;
  }
  LEDWrite(0.0, 0.1, 0.1); // Faint purple: finished with a run
  if (!Serial) {
    return;
  }

  Serial.print("Done! axialDist:");
  Serial.print(axialDist(), 4);
  Serial.print(",lateralDist:");
  Serial.print(lateralDist(), 4);
  Serial.print(",time:");
  float time = ((float)doneTime)/1000.0f;
  Serial.print(time, 3);
  Serial.print(",loopRate:");
  Serial.print(((float)loopCnt)/time, 0);
  Serial.print(",batteryVoltage:");
  Serial.print(batteryVoltage(), 1);
  Serial.println("V");
}