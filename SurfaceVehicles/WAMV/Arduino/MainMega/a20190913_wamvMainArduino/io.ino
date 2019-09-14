// Read in information from the kill Arduino, and assign mode of operation accordingly.  Note that the mainMega cannot actually kill the system, since the actual kill is handled by the killMega. 
void readKillArduino() {
  
  if (digitalRead(killCommPin) == LOW) {
    killStatus = 0;       // killMega sending LOW (UNKILL)
  }
  else {
    killStatus = 1;       // killMega sending HIGH (KILL)
  }
  
}

void readHandheldReceiver() {

  // Create local, non-volatile copy of pulse widths (QuickMedian does not like volatile arrays)
  int ch1PulseFilt[nMedian] = {};
  memcpy(ch1PulseFilt, ch1PulseArray, nMedian * 2);
  int ch2PulseFilt[nMedian] = {};
  memcpy(ch2PulseFilt, ch2PulseArray, nMedian * 2);
  int ch3PulseFilt[nMedian] = {};
  memcpy(ch3PulseFilt, ch3PulseArray, nMedian * 2);
  int ch5PulseFilt[nMedian] = {};
  memcpy(ch5PulseFilt, ch5PulseArray, nMedian * 2);
  int ch6PulseFilt[nMedian] = {};
  memcpy(ch6PulseFilt, ch6PulseArray, nMedian * 2);

  // Take median of pulse width arrays
  ch1PulseMedian = QuickMedian<int>::GetMedian(ch1PulseFilt, nMedian);
  ch2PulseMedian = QuickMedian<int>::GetMedian(ch2PulseFilt, nMedian);
  ch3PulseMedian = QuickMedian<int>::GetMedian(ch3PulseFilt, nMedian);
  ch5PulseMedian = QuickMedian<int>::GetMedian(ch5PulseFilt, nMedian);
  ch6PulseMedian = QuickMedian<int>::GetMedian(ch6PulseFilt, nMedian);
  
}

void readLowCurrentBatteryVoltage() {

  // Save low current battery read pin raw input (in bits)
  float voltLowCurrentBattBit = analogRead(voltLowCurrentPin);

  // Convert bits to voltage
  float lowCurrentVoltDivider = voltLowCurrentBattBit*5.06/1024;   // voltage at the voltage divider
  voltLowCurrentBatt = lowCurrentVoltDivider*4.1;                 // low current battery voltage

//  Serial.println(lowCurrentVoltDivider);
//  Serial.println(voltLowCurrentBatt);
   
}

void readMainBatteryVoltage() {

  String voltageMsg = "";

  // Get voltage measurement as a String
  for(int j=1; j<5; j++){
    voltageMsg += incoming[j];
  }

  // Convert voltage message to float
  previousVoltage = voltageMsg.toFloat();

  // Update voltage only when voltage measured changes
  if (voltMainBatt != previousVoltage && previousVoltage > 11.0){
    voltMainBatt = previousVoltage; 
  }
}

void determineMode() {
  
  // Determine mode from handheld receiver channel 3
  int switchUp = ch3PulseMin;
  int switchMid = round((ch3PulseMax+ch3PulseMin)/2);
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
  switchMid = round((ch5PulseMax+ch5PulseMin)/2);
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
  switchMid = round((ch6PulseMax+ch6PulseMin)/2);
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

  if (killStatus == 1) {
    changeLight(2);
  }
  else {
    if (mainBattCheck == 1) {
      if (lowCurrentBattCheck == 1) {
        // Flash white
        changeLight(1);
        delay(100);
        changeLight(0);
        delay(100);
      }
      else if (lowCurrentBattCheck == 2) {
        if (voltMainBatt > 29.4) {
          // Flash green
          changeLight(3);
          delay(100);
          changeLight(0);
          delay(100);
        }
        else if (voltMainBatt <= 28.42 && voltMainBatt > 27.44) {
          // Flash blue then green
          changeLight(3);
          delay(100);
          changeLight(4);
          delay(100);
        }
        else if (voltMainBatt <= 27.44 && voltMainBatt > 26.46) {
          // Flash blue
          changeLight(4);
          delay(100);
          changeLight(0);
          delay(100);
        }
        else if (voltMainBatt <= 26.46 && voltMainBatt > 25.48) {
          // Flash blue then red
          changeLight(4);
          delay(100);
          changeLight(2);
          delay(100);
        }
        else {
          // Flash red
          changeLight(2);
          delay(100);
          changeLight(0);
          delay(100);
        }
      }
      else {
        Serial.println("low current battery check error, WHITE");
        changeLight(1);
      }
    }
    else if (mainBattCheck == 2) {
      if (lowCurrentBattCheck == 1) {
        if (voltLowCurrentBatt > 16.24) {
          // Flash green
          changeLight(3);
          delay(100);
          changeLight(0);
          delay(100);
        }
        else if (voltLowCurrentBatt <= 16.24 && voltLowCurrentBatt > 15.56) {
          // Flash blue then green
          changeLight(3);
          delay(100);
          changeLight(4);
          delay(100);
        }
        else if (voltLowCurrentBatt <= 15.68 && voltLowCurrentBatt > 15.12) {
          // Flash blue
          changeLight(4);
          delay(100);
          changeLight(0);
          delay(100);
        }
        else if (voltLowCurrentBatt <= 15.12 && voltLowCurrentBatt > 14.56) {
          // Flash blue then red
          changeLight(4);
          delay(100);
          changeLight(2);
          delay(100);
        }
        else {
          // Flash red
          changeLight(2);
          delay(100);
          changeLight(0);
          delay(100);
        }
      }
      else if (lowCurrentBattCheck == 2) {
        if (mode == 1) {
          changeLight(5);
        }
        else if (mode == 2) {
          changeLight(4);
        }
        else {
          changeLight(1);
        }
      }
      else {
        Serial.println("low current battery check error, WHITE");
        changeLight(1);
      }
    }
    else {
      Serial.println("main battery check error, WHITE");
      changeLight(1);
    }
  }  
}

void changeLight(int lightColor) {
  
  // light color variable (0 = off, 1 = white, 2 = red, 3 = green, 4 = blue, 5 = yellow, 6 = light blue)
  
  if (lightColor == 0) {
    digitalWrite(redPin,HIGH);
    digitalWrite(bluePin,HIGH);
    digitalWrite(greenPin,HIGH);
  }
  else if (lightColor == 1) {
    digitalWrite(redPin,LOW);
    digitalWrite(bluePin,LOW);
    digitalWrite(greenPin,LOW);
  }
  else if (lightColor == 2) {
    digitalWrite(redPin,LOW);
    digitalWrite(bluePin,HIGH);
    digitalWrite(greenPin,HIGH);
  }
  else if (lightColor == 3) {
    digitalWrite(redPin,HIGH);
    digitalWrite(bluePin,HIGH);
    digitalWrite(greenPin,LOW);
  }
  else if (lightColor == 4) {
    digitalWrite(redPin,HIGH);
    digitalWrite(bluePin,LOW);
    digitalWrite(greenPin,HIGH);
  }  
  else if (lightColor == 5) {
    digitalWrite(redPin,LOW);
    digitalWrite(bluePin,HIGH);
    digitalWrite(greenPin,LOW);
  }
  else if (lightColor == 6) {
    digitalWrite(redPin,HIGH);
    digitalWrite(bluePin,LOW);
    digitalWrite(greenPin,LOW);
  }    
}

// Take joystick readings, and convert to setpoint thrust values
void joy2Setpoint() {

  // Grab ch1, ch4, and ch6 from filtering function
  int ch1Pulse = ch1PulseMedian;    // yaw (right stick left-right)
  int ch2Pulse = ch2PulseMedian;    // sway (left stick left-right)

  // Pulse width variables
  int ch1PulseNeutral = round((ch1PulseMax - ch1PulseMin) / 2) + ch1PulseMin;
  int ch2PulseNeutral = round((ch2PulseMax - ch2PulseMin) / 2) + ch2PulseMin;

  // Remove the deadzone
  if (ch1Pulse < ch1PulseNeutral + round(pulseDeadzone/2) && ch1Pulse > ch1PulseNeutral - round(pulseDeadzone/2)) {
    ch1Pulse = ch1PulseNeutral;
  }
  if (ch2Pulse < ch2PulseNeutral + round(pulseDeadzone/2) && ch2Pulse > ch2PulseNeutral - round(pulseDeadzone/2)) {
    ch2Pulse = ch2PulseNeutral;
  }

  // Map joystick inputs from -1000 to 1000
  int ch1Map = map(ch1Pulse,ch1PulseMin,ch1PulseMax,-1000,1000);  // surge (positive forward, negative backward)
  int ch2Map = map(ch2Pulse,ch2PulseMin,ch2PulseMax,1000,-1000);  // yaw (positive CCW, negative CW)

  // Calculate surge sway and yaw components
  int surgeLeft = ch1Map;     // surge (forward positive)
  int surgeRight = ch1Map;
  int yawLeft = -ch2Map;       // yaw (CCW positive)
  int yawRight = ch2Map;

  // Map thruster components from -1000 to 1000
  leftThrusterSetpoint = constrain(surgeLeft + yawLeft,-1000,1000);
  rightThrusterSetpoint = constrain(surgeRight + yawRight,-1000,1000);

  // Create messages for I2C comms
  getMsgs();
  msgReset();
  
}

String commandToMsg(int motor){
  /***
   *  Converts motor commands into serial messages. Checks to see if
   *  the motor command is greater/less than neutral point to determine
   *  whether F/R (forward/reverse) direction character should be used. 
   *  
   *  Inputs:
   *   - motor <int>: motor command
   *   
   *  Return:
   *   - msg <String>: motor magnitude and direction as a String
   */
  String msg; 
  if (abs(motor) < 1000 && abs(motor) >= 100){
    msg = "0" + String(abs(motor));
  }
  else if(abs(motor) < 100 && abs(motor) >=10){
    msg = "00" + String(abs(motor));
  }
  else if(abs(motor) < 10){
    msg = "000" + String(abs(motor));
  }
  if (motor < 0 && motor >= -1000){
    dir = 'R';
    return msg += dir;
  }
  if (motor > 0 && motor <= 1000){
    dir = 'F';
    return msg += dir;
  }
  else{
    dir = 'N';
    return msg += dir;
  }
}

void msgReset(){
  //Function to reset the serial messages after each loop.  
  q1Msg = "Q1";
  q2Msg = "Q2";
  q3Msg = "Q3";
  q4Msg = "Q4";
  temp = "";
}

void getMsgs(){
  q1Msg += commandToMsg(leftThrusterSetpoint);   // Q1 will act as rightServoOut
  q2Msg += commandToMsg(rightThrusterSetpoint);  // Q2 will act as leftServoOut
//  q3Msg += commandToMsg(q3Out);  
//  q4Msg += commandToMsg(q4Out); 
  
  temp += q1Msg; 
  temp += q2Msg; 
//  temp += q3Msg;
//  temp += q4Msg;
  
  // Ensure message being sent is correct size
  if(sizeof(temp) == 6){
    motorCmds = temp;
  }
}
