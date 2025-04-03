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

void loop() {
  waitForStart();
  bool done = false;
  while (!done) {
    loopLocalization();
    done = loopControl();
  }
}
