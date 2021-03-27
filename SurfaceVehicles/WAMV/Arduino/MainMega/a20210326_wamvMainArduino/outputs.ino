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

  // Determine calibration check mode from receiver channel 5
  switchUp = ch5PulseMin;
  switchMid = round((ch5PulseMax + ch5PulseMin) / 2);
  switchDown = ch5PulseMax;
  if (ch5PulseMedian + pulseTolerance > switchUp && ch5PulseMedian - pulseTolerance < switchUp) {
    // Channel 5 switch is in the up position (yes to calibration check)
    //mainBattCheck = 1;  *THIS CHANNEL WAS TEMPORARILY USED FOR MAIN BATTERY CHECK. NOT RIGHT NOW*
    calibrationCheck = 1; 
  }
  else if (ch5PulseMedian + pulseTolerance > switchMid && ch5PulseMedian - pulseTolerance < switchMid) {
    // Channel 5 switch is in the middle position (no to main battery check)
    //mainBattCheck = 2;
    calibrationCheck = 2; 
  }
  else if (ch5PulseMedian + pulseTolerance > switchDown && ch5PulseMedian - pulseTolerance < switchDown) {
    // Channel 5 switch is in the down position (no to main battery check)
    //mainBattCheck = 2;
    calibrationCheck = 2;
  }
  else {
    // Invalid input from channel 5
    Serial.println("Calibration check switch (channel 5) reporting invalid input; check transmitter connection...");
    //mainBattCheck = 0;
    calibrationCheck = 0;
  }

  // Determine software reset check mode from receiver channel 6
  switchUp = ch6PulseMin;
  switchMid = round((ch6PulseMax + ch6PulseMin) / 2);
  switchDown = ch6PulseMax;
  if (ch6PulseMedian + pulseTolerance > switchUp && ch6PulseMedian - pulseTolerance < switchUp) {
    // Channel 6 switch is in the up position (yes to software reset check)
    //lowCurrentBattCheck = 1;
    softwareResetCheck = 1; 
  }
  else if (ch6PulseMedian + pulseTolerance > switchMid && ch6PulseMedian - pulseTolerance < switchMid) {
    // Channel 6 switch is in the middle position (no to software reset check)
    //lowCurrentBattCheck = 2;
    softwareResetCheck = 2; 
  }
  else if (ch6PulseMedian + pulseTolerance > switchDown && ch6PulseMedian - pulseTolerance < switchDown) {
    // Channel 6 switch is in the down position (no to software reset check)
    //lowCurrentBattCheck = 2;
    softwareResetCheck = 2; 
  }
  else {
    // Invalid input from channel 6
    Serial.println("Software reset check switch (channel 6) reporting invalid input; check transmitter connection...");
    //lowCurrentBattCheck = 0;
    softwareResetCheck = 0; 
  }

}

void calibrateController(){
  /*
   *  Sometimes the remote controller's receiver changes its max and minimum PWM signals, which affects our manual operation.
   *  This code is meant to help us recalibrate the expected PWM signals on the fly. 
   */

   int tempCh1 = 0;   // temporary variable to store current ch1 PWM value
   int tempCh2 = 0;   // temporary variable to store current ch2 PWM value
   int tempTol = 110; 

   //Check if we want to recalibrate expected PWM signals from the receiver module
   if (calibrationCheck == 1){
     tempCh1 = ch1PulseMedian;  // store current filtered ch1 PWM 
     tempCh2 = ch2PulseMedian;  // store current filtered ch2 PWM

     // Recalibrate ch1PulseMax and ch1PulseMin values
     if (tempCh1 >= ch1PulseMax - tempTol && tempCh1 <= ch1PulseMax + tempTol) {
      ch1PulseMax = tempCh1;
     }
     else if(tempCh1 >= ch1PulseMin - tempTol && tempCh1 <= ch1PulseMin + tempTol){
      ch1PulseMin = tempCh1;
     }

    // Recalibrate ch2PulseMax and ch2PulseMin values
    if (tempCh2 >= ch2PulseMax - tempTol && tempCh2 <= ch2PulseMax + tempTol) {
      ch2PulseMax = tempCh2;
     }
     else if(tempCh2 >= ch2PulseMin - tempTol && tempCh2 <= ch2PulseMin + tempTol){
      ch2PulseMin = tempCh2;
     }
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
      //battCheck = battCheckLight();
      if (battCheck == 0) {
        // Battery check did not occur, we are in manual mode, change light to solid yellow
        changeLight(5);
      }
    }
    else if (mode == 2) {
      // Autonomous mode engaged, proceed to check battery mode
      //battCheck = battCheckLight();
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

//bool battCheckLight() {
//  /*
//     Battery check light function checks the battery mode.
//     Does the same function as changeLight, but specific to checking the battery check mode to keep that code from containing too many nested if statements.
//  */
//
//  // Battery check variable
//  bool battCheck = 0;
//
//  if (mainBattCheck == 1) {
//    if (lowCurrentBattCheck == 1) {
//      // Main battery check and low battery check mode simultaneously. Flash white to indicate user error in selecting both modes at the same time.
//      changeLight(1);
//      delay(100);
//      changeLight(0);
//      delay(100);
//      battCheck = 1;
//    }
//    else {
//      // Main battery check mode, flash light as appropriate to main battery level. Scales between 21V (dead) to 29.4V (full).
//      if (voltMainBatt > 27.72) {
//        // 80% and above, flash green.
//        changeLight(3);
//        delay(100);
//        changeLight(0);
//        delay(100);
//        battCheck = 1;
//      }
//      else if (voltMainBatt <= 27.72 && voltMainBatt > 26.04) {
//        // 60% to 80% capcity, flash blue then green.
//        changeLight(3);
//        delay(100);
//        changeLight(4);
//        delay(100);
//        battCheck = 1;
//      }
//      else if (voltMainBatt <= 26.04 && voltMainBatt > 24.36) {
//        // 40% to 60% capacity, flash blue.
//        changeLight(4);
//        delay(100);
//        changeLight(0);
//        delay(100);
//        battCheck = 1;
//      }
//      else if (voltMainBatt <= 24.36 && voltMainBatt > 22.68) {
//        // 20% to 40% capacity, flash blue then red.
//        changeLight(4);
//        delay(100);
//        changeLight(2);
//        delay(100);
//        battCheck = 1;
//      }
//      else {
//        // Bottom 20% of battery, flash red
//        changeLight(2);
//        delay(100);
//        changeLight(0);
//        delay(100);
//        battCheck = 1;
//      }
//    }
//  }
//  else {
//    if (lowCurrentBattCheck == 1) {
//      // Low current battery check mode, flash light as appropriate to low current battery level. Scales between 12V (dead) to 16.8V (full)
//      if (voltLowCurrentBatt > 15.84) {
//        // 80% and above, flash green.
//        delay(100);
//        changeLight(0);
//        delay(100);
//        battCheck = 1;
//      }
//      else if (voltLowCurrentBatt <= 15.84 && voltLowCurrentBatt > 14.88) {
//        // 60% to 80% capcity, flash blue then green.
//        changeLight(3);
//        delay(100);
//        changeLight(4);
//        delay(100);
//        battCheck = 1;
//      }
//      else if (voltLowCurrentBatt <= 14.88 && voltLowCurrentBatt > 13.92) {
//        // 40% to 60% capacity, flash blue.
//        changeLight(4);
//        delay(100);
//        changeLight(0);
//        delay(100);
//        battCheck = 1;
//      }
//      else if (voltLowCurrentBatt <= 13.92 && voltLowCurrentBatt > 12.96) {
//        // 20% to 40% capacity, flash blue then red.
//        changeLight(4);
//        delay(100);
//        changeLight(2);
//        delay(100);
//        battCheck = 1;
//      }
//      else {
//        // Bottom 20% of battery, flash red
//        changeLight(2);
//        delay(100);
//        changeLight(0);
//        delay(100);
//        battCheck = 1;
//      }
//    }
//  }
//  return battCheck;
//}

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
     Create message to send over I2C (based on the thruster setpoints)
  */

  // Create string message to send over I2C
  q1Msg += commandToMsg(leftThrusterSetpoint);   // Q1 will act as rightServoOut
  q2Msg += commandToMsg(rightThrusterSetpoint);  // Q2 will act as leftServoOut
  //  q3Msg += commandToMsg(q3Out);
  //  q4Msg += commandToMsg(q4Out);

  // Print debug statements
  //    Serial.print("q1Msg: ");
  //    Serial.println(q1Msg);
  //    Serial.print("q2Msg: ");
  //    Serial.println(q2Msg);

  // Ensure string message being set is correct size
  tempMsg += q1Msg;
  tempMsg += q2Msg;
  //  temp += q3Msg;
  //  temp += q4Msg;
  if (sizeof(tempMsg) == 6) {
    motorCmds = tempMsg;
  }

  // Reset global serial messages
  dir = 'N';
  q1Msg = "Q1";
  q2Msg = "Q2";
  q3Msg = "Q3";
  q4Msg = "Q4";
  tempMsg = "";

}

// Create string to send over I2C based on thruster setpoints
String commandToMsg(int motor) {
  /*
     Create string to send over I2C based on thruster setpoints.
     Checks to see if the motor command is greater/less than neutral point to determine whether F/R (forward/reverse) direction character should be used.

     Arguments
       int motor: motor command

     Output:
       String msg: motor magnitude and direction as a String
  */

  // Create output variable
  String msg;

  // Logic for building string
  if (abs(motor) < 1000 && abs(motor) >= 100) {
    msg = "0" + String(abs(motor));
  }
  else if (abs(motor) < 100 && abs(motor) >= 10) {
    msg = "00" + String(abs(motor));
  }
  else if (abs(motor) < 10) {
    msg = "000" + String(abs(motor));
  }
  else {
    msg = String(abs(motor));
  }
  if (motor < 0 && motor >= -1000) {
    dir = 'R';
    return msg += dir;
  }
  if (motor > 0 && motor <= 1000) {
    dir = 'F';
    return msg += dir;
  }
  else {
    dir = 'N';
    return msg += dir;
  }

}

void softwareReset(){
  /*
   * This functions performs a software reset if the software reset switch is in the "UP" position.
   */

   if (softwareResetCheck == 1){
    //watchdog.reset();
   }
}
