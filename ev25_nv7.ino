void setup() {
  setupHardware();
  setupSensing();
}

void waitForStart() {
  // Wait for button to get pressed
  while (!GOPressed()) {
    
    motorWriteL(0.0);
    motorWriteR(0.0);
    printResults();
    delay(50);
  }
  while (GOPressed()) {
    LEDWrite(0, 1, 0);
  }
  resetLocalization();
}

long lastPrint = 0;
void loop() {
  loopLocalization();

  if (millis() - lastPrint > 50) {
    Serial.print("gyro:");
    Serial.print(readGyro()*RAD_TO_DEG);
    Serial.print(",angleL:");
    Serial.print(readAngleL());
    Serial.print(",angleR:");
    Serial.println(readAngleR());
    lastPrint = millis();
  }
}
