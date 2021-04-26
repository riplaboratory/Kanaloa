void determineMode() {
  /*
     Determine operation mode (kill, battery check, and auto/manual) based on inputs
  */

  // Determine mode from handheld receiver channel 3
  int switchUp = ch3PulseMin;
  int switchMid = round((ch3PulseMax + ch3PulseMin) / 2);
  int switchDown = ch3PulseMax;
  if (ch3PulseMedian + pulseTolerance > switchUp && ch3PulseMedian - pulseTolerance < switchUp) {
    // Channel 3 switch is in the up position (autonomous)
    mode = 2;
  }
  else if (ch3PulseMedian + pulseTolerance > switchMid && ch3PulseMedian - pulseTolerance < switchMid) {
    // Channel 3 switch is in the middle position (manual)
    mode = 1;
  }
  else if (ch3PulseMedian + pulseTolerance > switchDown && ch3PulseMedian - pulseTolerance < switchDown) {
    // Channel 3 switch is in the down position (manual)
    mode = 1;
  }
  else {
    // Invalid input from channel 3
    Serial.println("Mode switch (channel 3) reporting invalid input; check transmitter connection...");
    mode = 0;
  }

  // Determine main battery check mode from receiver channel 5
  switchUp = ch5PulseMin;
  switchMid = round((ch5PulseMax + ch5PulseMin) / 2);
  switchDown = ch5PulseMax;
  if (ch5PulseMedian + pulseTolerance > switchUp && ch5PulseMedian - pulseTolerance < switchUp) {
    // Channel 5 switch is in the up position (yes to main battery check)
    mainBattCheck = 1;
  }
  else if (ch5PulseMedian + pulseTolerance > switchMid && ch5PulseMedian - pulseTolerance < switchMid) {
    // Channel 5 switch is in the middle position (no to main battery check)
    mainBattCheck = 2;
  }
  else if (ch5PulseMedian + pulseTolerance > switchDown && ch5PulseMedian - pulseTolerance < switchDown) {
    // Channel 5 switch is in the down position (no to main battery check)
    mainBattCheck = 2;
  }
  else {
    // Invalid input from channel 5
    Serial.println("Main battery check switch (channel 5) reporting invalid input; check transmitter connection...");
    mainBattCheck = 0;
  }

  // Determine main battery check mode from receiver channel 6
  switchUp = ch6PulseMin;
  switchMid = round((ch6PulseMax + ch6PulseMin) / 2);
  switchDown = ch6PulseMax;
  if (ch6PulseMedian + pulseTolerance > switchUp && ch6PulseMedian - pulseTolerance < switchUp) {
    // Channel 6 switch is in the up position (yes to low current battery check)
    lowCurrentBattCheck = 1;
  }
  else if (ch6PulseMedian + pulseTolerance > switchMid && ch6PulseMedian - pulseTolerance < switchMid) {
    // Channel 6 switch is in the middle position (no to low current battery check)
    lowCurrentBattCheck = 2;
  }
  else if (ch6PulseMedian + pulseTolerance > switchDown && ch6PulseMedian - pulseTolerance < switchDown) {
    // Channel 6 switch is in the down position (no to low current battery check)
    lowCurrentBattCheck = 2;
  }
  else {
    // Invalid input from channel 6
    Serial.println("Low current battery check switch (channel 6) reporting invalid input; check transmitter connection...");
    lowCurrentBattCheck = 0;
  }

}

void controlLight() {
  /*
     Changes light in response to mode of operation.
  */

  int battCheck = 0;  // variable indicating whether battery check should occur

  // Check modes for light combo
  if (killStatus == 0) {
    // System is unkilled, proceed to check mode
    if (mode == 1) {
      // Manual mode engaged, proceed to check battery mode
      battCheck = battCheckLight();
      if (battCheck == 0) {
        // Battery check did not occur, we are in manual mode, change light to solid yellow
        changeLight(5);
      }
    }
    else if (mode == 2) {
      // Autonomous mode engaged, proceed to check battery mode
      battCheck = battCheckLight();
      if (battCheck == 0) {
        // Battery check did not occur, we are in autnomous mode, change light to solid blue
        changeLight(4);
      }
    }
    else {
      // Mode read error, flash white.
      changeLight(1);
      delay(50);
      changeLight(0);
      delay(50);
    }
  }
  else {
    // System is killed solid red
    changeLight(2);
  }
}

bool battCheckLight() {
  /*
     Battery check light function checks the battery mode.
     Does the same function as changeLight, but specific to checking the battery check mode to keep that code from containing too many nested if statements.
  */

  // Battery check variable
  bool battCheck = 0;

  if (mainBattCheck == 1) {
    if (lowCurrentBattCheck == 1) {
      // Main battery check and low battery check mode simultaneously. Flash white to indicate user error in selecting both modes at the same time.
      changeLight(1);
      delay(100);
      changeLight(0);
      delay(100);
      battCheck = 1;
    }
    else {
      // Main battery check mode, flash light as appropriate to main battery level. Scales between 21V (dead) to 29.4V (full).
      if (voltMainBatt > 27.72) {
        // 80% and above, flash green.
        changeLight(3);
        delay(100);
        changeLight(0);
        delay(100);
        battCheck = 1;
      }
      else if (voltMainBatt <= 27.72 && voltMainBatt > 26.04) {
        // 60% to 80% capcity, flash blue then green.
        changeLight(3);
        delay(100);
        changeLight(4);
        delay(100);
        battCheck = 1;
      }
      else if (voltMainBatt <= 26.04 && voltMainBatt > 24.36) {
        // 40% to 60% capacity, flash blue.
        changeLight(4);
        delay(100);
        changeLight(0);
        delay(100);
        battCheck = 1;
      }
      else if (voltMainBatt <= 24.36 && voltMainBatt > 22.68) {
        // 20% to 40% capacity, flash blue then red.
        changeLight(4);
        delay(100);
        changeLight(2);
        delay(100);
        battCheck = 1;
      }
      else {
        // Bottom 20% of battery, flash red
        changeLight(2);
        delay(100);
        changeLight(0);
        delay(100);
        battCheck = 1;
      }
    }
  }
  else {
    if (lowCurrentBattCheck == 1) {
      // Low current battery check mode, flash light as appropriate to low current battery level. Scales between 12V (dead) to 16.8V (full)
      if (voltLowCurrentBatt > 15.84) {
        // 80% and above, flash green.
        delay(100);
        changeLight(0);
        delay(100);
        battCheck = 1;
      }
      else if (voltLowCurrentBatt <= 15.84 && voltLowCurrentBatt > 14.88) {
        // 60% to 80% capcity, flash blue then green.
        changeLight(3);
        delay(100);
        changeLight(4);
        delay(100);
        battCheck = 1;
      }
      else if (voltLowCurrentBatt <= 14.88 && voltLowCurrentBatt > 13.92) {
        // 40% to 60% capacity, flash blue.
        changeLight(4);
        delay(100);
        changeLight(0);
        delay(100);
        battCheck = 1;
      }
      else if (voltLowCurrentBatt <= 13.92 && voltLowCurrentBatt > 12.96) {
        // 20% to 40% capacity, flash blue then red.
        changeLight(4);
        delay(100);
        changeLight(2);
        delay(100);
        battCheck = 1;
      }
      else {
        // Bottom 20% of battery, flash red
        changeLight(2);
        delay(100);
        changeLight(0);
        delay(100);
        battCheck = 1;
      }
    }
  }
  return battCheck;
}

void changeLight(int lightColor) {
  /*
     Changes the light color based on input "lightColor")
  */

  // light color variable (0 = off, 1 = white, 2 = red, 3 = green, 4 = blue, 5 = yellow, 6 = light blue)

  if (lightColor == 0) {
    digitalWrite(redPin, HIGH);
    digitalWrite(bluePin, HIGH);
    digitalWrite(greenPin, HIGH);
  }
  else if (lightColor == 1) {
    digitalWrite(redPin, LOW);
    digitalWrite(bluePin, LOW);
    digitalWrite(greenPin, LOW);
  }
  else if (lightColor == 2) {
    digitalWrite(redPin, LOW);
    digitalWrite(bluePin, HIGH);
    digitalWrite(greenPin, HIGH);
  }
  else if (lightColor == 3) {
    digitalWrite(redPin, HIGH);
    digitalWrite(bluePin, HIGH);
    digitalWrite(greenPin, LOW);
  }
  else if (lightColor == 4) {
    digitalWrite(redPin, HIGH);
    digitalWrite(bluePin, LOW);
    digitalWrite(greenPin, HIGH);
  }
  else if (lightColor == 5) {
    digitalWrite(redPin, LOW);
    digitalWrite(bluePin, HIGH);
    digitalWrite(greenPin, LOW);
  }
  else if (lightColor == 6) {
    digitalWrite(redPin, HIGH);
    digitalWrite(bluePin, LOW);
    digitalWrite(greenPin, LOW);
  }
}

void joy2Setpoint() {
  /*
     Take readings fromt receiver joystick channels, and convert to setpoint thruster values from -1000 to 1000.
  */

  // Grab ch1 and ch2 from filtering function
  int ch1Pulse = ch1PulseMedian;    // yaw (right stick left-right)
  int ch2Pulse = ch2PulseMedian;    // sway (left stick left-right)

  // Pulse width variables
  int ch1PulseNeutral = round((ch1PulseMax - ch1PulseMin) / 2) + ch1PulseMin;
  int ch2PulseNeutral = round((ch2PulseMax - ch2PulseMin) / 2) + ch2PulseMin;

  // Remove the deadzone
  if (ch1Pulse < ch1PulseNeutral + round(pulseDeadzone / 2) && ch1Pulse > ch1PulseNeutral - round(pulseDeadzone / 2)) {
    ch1Pulse = ch1PulseNeutral;
  }
  if (ch2Pulse < ch2PulseNeutral + round(pulseDeadzone / 2) && ch2Pulse > ch2PulseNeutral - round(pulseDeadzone / 2)) {
    ch2Pulse = ch2PulseNeutral;
  }

  // Map joystick inputs from -1000 to 1000
  int ch1Map = map(ch1Pulse, ch1PulseMin, ch1PulseMax, -1000, 1000); // surge (positive forward, negative backward)
  int ch2Map = map(ch2Pulse, ch2PulseMin, ch2PulseMax, 1000, -1000); // yaw (positive CCW, negative CW)

  // Calculate surge sway and yaw components
  int surgeLeft = ch1Map;     // surge (forward positive)
  int surgeRight = ch1Map;
  int yawLeft = -ch2Map;      // yaw (CCW positive)
  int yawRight = ch2Map;

  // Map thruster components from -1000 to 1000
  leftThrusterSetpoint = constrain(surgeLeft + yawLeft, -1000, 1000);
  rightThrusterSetpoint = constrain(surgeRight + yawRight, -1000, 1000);
  
}

void zeroSetpoints() {
  /*
     Sets the thruster setpoints to zero.  This should be used when the system is killed to prevent the high current arduino controller from going crazy while system is killed.
  */

  // Set thruster components to zero
  leftThrusterSetpoint = 0;
  rightThrusterSetpoint = 0;

  // Create messages for I2C comms
  createI2cMsg();

}

void createI2cMsg() {
  /*
   *  Create message to send over I2C (based on the thruster setpoints)
   */
   
  // Determine which direction the motors should rotate
  q1Dir = determineMotorDir(leftThrusterSetpoint);      // Q1 will act as leftServoOut
  q2Dir = determineMotorDir(rightThrusterSetpoint);     // Q2 will act as rightServoOut

  // Update motorMsg with directions
  motorMsg[6] = q1Dir;
  motorMsg[13] = q2Dir;

  // Convert the int motor values into c-strings
  leftThrusterSetpoint = abs(leftThrusterSetpoint);
  rightThrusterSetpoint = abs(rightThrusterSetpoint);
  convertToStr();

  // Create one long message with both motor commands in it
  for (int i=0; i<4; i++){
    motorMsg[i+2] = cQ1Motor[i]; 
    motorMsg[i+9] = cQ2Motor[i];
  }
}

char determineMotorDir(int motor){ 
  // Determine which direction the motor should rotate based on motor speed values

  char dir = 'N'; // placeholder for character to return
  
  if (motor < 0 && motor >= -1000){
    dir = 'R';
  }
  else if (motor > 0 && motor <= 1000){
    dir = 'F'; 
  }
  else{
    dir = 'N'; 
  }

  return dir;
}

// Create string to send over I2C based on thruster setpoints
void convertToStr(){
  // Convert the motor commands from integers to C-strings
  // Ensure they are always the same length (i.e. 900 would be '0900' to maintain length of four chars)

  itoa(leftThrusterSetpoint, temp, 10); 

  if (leftThrusterSetpoint == 1000){
    cQ1Motor[0] = '1';
    cQ1Motor[1] = '0';
    cQ1Motor[2] = '0';
    cQ1Motor[3] = '0';
  }
  else if (100 <= leftThrusterSetpoint && leftThrusterSetpoint < 1000){
    cQ1Motor[0] = '0';
    for (int i=0; i<=2; i++){
      cQ1Motor[i+1] = temp[i];
    }
  }
  else if (10 <= leftThrusterSetpoint && leftThrusterSetpoint < 100){
    cQ1Motor[0] = '0';
    cQ1Motor[1] = '0';
    for (int i=0; i<=1; i++){
      cQ1Motor[i+2] = temp[i];
    }
  }

  else if (0 <= leftThrusterSetpoint && leftThrusterSetpoint < 10){
    cQ1Motor[0] = '0';
    cQ1Motor[1] = '0';
    cQ1Motor[2] = '0';
    for (int i=0; i<=0; i++){
      cQ1Motor[i+3] = temp[i];
    }
  }

  
  itoa(rightThrusterSetpoint, temp, 10); 

  if (rightThrusterSetpoint == 1000){
    cQ2Motor[0] = '1';
    cQ2Motor[1] = '0';
    cQ2Motor[2] = '0';
    cQ2Motor[3] = '0';
  }
  else if (100 <= rightThrusterSetpoint && rightThrusterSetpoint < 1000){
    cQ2Motor[0] = '0';
    for (int i=0; i<=2; i++){
      cQ2Motor[i+1] = temp[i];
    }
  }
  else if (10 <= rightThrusterSetpoint && rightThrusterSetpoint < 100){
    cQ2Motor[0] = '0';
    cQ2Motor[1] = '0';
    for (int i=0; i<=1; i++){
      cQ2Motor[i+2] = temp[i];
    }
  }

  else if (0 <= rightThrusterSetpoint && rightThrusterSetpoint < 10){
    cQ2Motor[0] = '0';
    cQ2Motor[1] = '0';
    cQ2Motor[2] = '0';
    for (int i=0; i<=0; i++){
      cQ2Motor[i+3] = temp[i];
    }
  }
}
