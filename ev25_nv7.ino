void setup() {
  setupHardware();
  setupSensing();
}

void waitForStart() {
  // Wait for button to get pressed
  while (!GOPressed()) {
    if (STOPPressed()) {
      LEDWrite(1, 0, 0);
    } else {
      printResults();
    }
    motorWriteL(0.0);
    motorWriteR(0.0);
    delay(50);
  }
  while (GOPressed()) {
    LEDWrite(0, 1, 0);
  }
  resetLocalization();
}

void loop() {
  waitForStart();
  bool end = false;
  while (!end) {
    loopLocalization();
    end = loopControl();
    if (STOPPressed()) {
      done();
      end = true;
    }
  }
}
