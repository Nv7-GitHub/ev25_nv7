const float P_forward = 1.5;
const float P_difference = 2.0;
const float D = 0.06;
const float D_forward = 0.15;
const float maxSpeed = 1.0;
const float accelTime = 0.3;
float filteredOmega = 0;

long lastPrint = 0;

bool loopControl() {
  float distRem = targetDist() - axialDist();
  if (abs(distRem) < 0.003 && linVel() < 0.01) { // 0.3cm
    done();
    return true;
  }

  // Forward PID controller
  float pow = P_forward * distRem;
  if (abs(distRem) > 0.02) { // 2 cm
    pow = P_forward * distRem;
  } else {
    pow = P_forward * distRem; // When within 2cm of the end, use a stronger P controller
    float mult = sqrt(0.003/min(distRem, 0.003)); // Make P proportional to distance from end, with sqrt to make it stronger but less jerky when super close
    pow *= mult*4 + 1;
  }

  // D term
  pow -= D_forward*linVel();

  // Friction compensation (aka feedforward)
  if (pow > 0) {
    pow += 0.1;
  } else {
    pow -= 0.1;
  }

  // Make sure it follows max speed
  bool atMax = false;
  if (pow > maxSpeed) {
    pow = maxSpeed;
    atMax = true;
  } else if (pow < -maxSpeed) {
    pow = -maxSpeed;
    atMax = true;
  }

  // Accel curve
  float prog = min(getTime()/accelTime, 1.0);
  pow *= prog;
  if (getTime() < accelTime) {
    LEDWrite(prog, prog, 0); // Accel phase: yellow
  } else if (atMax) {
    LEDWrite(1, 0, 1); // Full speed phase: pink
  } else {
    LEDWrite(0, 0, 1); // Decel phase: blue
  }

  // Calculate target angle
  float ang;
  if (distRem > 0.4) {
    ang = heading() - atan2(0-lateralDist(), distRem);
  } else {
    ang = heading();
  }

  // Calculate W (angular control output)
  float w = ang * P_difference;
  filteredOmega = filteredOmega * 0.3 + angVel() * 0.7; // Low-pass filter the ang. vel.
  w += filteredOmega * D;

  // Apply W
  float powL = pow + w;
  float powR = pow - w;

  // Scale so that they don't pass 1
  if (abs(powL) > 1.0 || abs(powR) > 1.0) {
    if (abs(powL) > abs(powR)) {
      powL /= abs(powL);
      powR /= abs(powL);
    } else {
      powL /= abs(powR);
      powR /= abs(powR);
    }
  }

  // Powers
  motorWriteL(powL);
  motorWriteR(powR);

  // Debugging
  if (millis() - lastPrint > 50) {
    Serial.print("time:");
    Serial.print(getTime());
    Serial.print(",axialDist:");
    Serial.print(axialDist(), 4);
    Serial.print(",lateralDist:");
    Serial.print(lateralDist(), 4);
    Serial.print(",angVel:");
    Serial.print(angVel());
    Serial.print(",powL:");
    Serial.print(powL);
    Serial.print(",powR:");
    Serial.print(powR);
    Serial.print(",ang:");
    Serial.print(ang * RAD_TO_DEG);
    Serial.print(",vel:");
    Serial.println(linVel());
    lastPrint = millis();
  }

  return false;
}