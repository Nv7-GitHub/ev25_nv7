const float P_forward = 0.8;
const float P_difference = 2.0;
const float D = 0.15;
const float maxSpeed = 1.0;
float filteredOmega = 0;

long lastPrint = 0;

bool loopControl() {
  float distRem = targetDist() - axialDist();
  if (abs(distRem) < 0.003 && linVel() < 0.1) { // 0.3cm
    done();
    return true;
  }

  // Forward PID controller
  float pow;
  if (abs(distRem) > 0.02) { // 2 cm
    pow = P_forward * distRem;
  } else {
    pow = P_forward * 3 * distRem; // When within 2cm of the end, use a stronger P controller
  }

  // Friction compensation (aka feedforward)
  if (pow > 0) {
    pow += 0.1;
  } else {
    pow -= 0.1;
  }

  // Make sure it follows max speed
  bool maxSpeed = false;
  if (pow > maxSpeed) {
    pow = maxSpeed;
    maxSpeed = true;
  } else if (pow < -maxSpeed) {
    pow = -maxSpeed;
    maxSpeed = true;
  }

  // Accel curve for 500ms
  float prog = min(getTime()/0.5, 1.0);
  pow *= prog;
  if (getTime() < 0.5) {
    LEDWrite(prog, prog, 0); // Accel phase: yellow
  } else if (maxSpeed) {
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
    Serial.print(axialDist());
    Serial.print(",lateralDist:");
    Serial.print(lateralDist());
    Serial.print(",angVel:");
    Serial.print(angVel());
    Serial.print(",powL:");
    Serial.print(powL);
    Serial.print(",powR:");
    Serial.println(powL);
    lastPrint = millis();
  }

  return false;
}