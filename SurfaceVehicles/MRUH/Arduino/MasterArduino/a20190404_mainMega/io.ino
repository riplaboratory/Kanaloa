// Read kill state inputs 
void readKill() {

  // Read kill inputs
  ch7Pulse = pulseIn(ch7Pin,HIGH);              // ch 7 (remote kill switch channel)
  int physKillRaw = digitalRead(physKillPin);   // physical kill switch channel

  // Determine remote kill swtich state (ch7)
  if (ch7Pulse > ch7PulseMax - pulseTolerance && ch7Pulse < ch7PulseMax + pulseTolerance) {
    remKillStatus = 0;      // unkill
  }
  else if (ch7Pulse > ch7PulseMin - pulseTolerance && ch7Pulse < ch7PulseMin + pulseTolerance) {
    remKillStatus = 1;      // kill
  }
  else { 
    Serial.println("Error reading remote kill swtich signal (ch7) from receiver...killing");
    remKillStatus = 2;      // error
  }

  // Determine physical kill swtich state (from physical kill buttons
  if (physKillRaw == 1) {
    physKillStatus = 0;     // unkill
  }
  else if (physKillRaw == 0) {
    physKillStatus = 1;     // kill
  }
  else {
    Serial.println("Error reading remote physical kill state...this state should be impossible to trip");
    physKillStatus = 2;     // error
  }

  // Determine kill status
  if (remKillStatus == 0 && physKillStatus == 0) {
    killStatus = 0;     // unkill
  }
  else {
    killStatus = 1;     // kill
  }
  
}

// Read joystick inputs
void joy2Setpoint() {
  
  // Read joystick inputs
  ch1Pulse = pulseIn(ch1Pin,HIGH);    // ch 1 (surge joystick channel)
  ch2Pulse = pulseIn(ch2Pin,HIGH);    // ch 2 (sway joystick channel)
  ch3Pulse = pulseIn(ch3Pin,HIGH);    // ch 3 (yaw joystick channel)

  // Map joystick inputs from -1000 to 1000
  int ch1Map = map(ch1Pulse,ch1PulseMin,ch1PulseMax,-1000,1000);
  int ch2Map = map(ch2Pulse,ch2PulseMin,ch2PulseMax,-1000,1000);
  int ch3Map = map(ch3Pulse,ch3PulseMin,ch3PulseMax,-1000,1000);

  // Remove the deadzone
  if (ch1Map < pulseDeadzone && ch1Map > -pulseDeadzone) {
    ch1Map = 0;
  }
  if (ch2Map < pulseDeadzone && ch2Map > -pulseDeadzone) {
    ch2Map = 0;
  }
  if (ch3Map < pulseDeadzone && ch3Map > -pulseDeadzone) {
    ch3Map = 0;
  }

  // Calculate surge sway and yaw components
  int surgeQ1 = ch1Map;   // surge (forward positive)
  int surgeQ2 = ch1Map;
  int surgeQ3 = ch1Map;
  int surgeQ4 = ch1Map;
  int swayQ1 = -ch2Map;   // sway (right positive)
  int swayQ2 = ch2Map;
  int swayQ3 = -ch2Map;
  int swayQ4 = ch2Map;
  int yawQ1 = -ch3Map;    // yaw (CLOCKWISE positive)
  int yawQ2 = ch3Map;
  int yawQ3 = ch3Map;
  int yawQ4 = -ch3Map;

  // Map thruster components from -1000 to 1000
  q1Setpoint = constrain(surgeQ1 + swayQ1 + yawQ1,-1000,1000);
  q2Setpoint = constrain(surgeQ2 + swayQ2 + yawQ2,-1000,1000);
  q3Setpoint = constrain(surgeQ3 + swayQ3 + yawQ3,-1000,1000);
  q4Setpoint = constrain(surgeQ4 + swayQ4 + yawQ4,-1000,1000);
  
}

// Calculate acceleration-limited outputs as a function of the setpoint and send to thrusters
void setpoint2Output() {

  // Calculate time delta since last loop
  timeNow = millis();
  float timeDelta = (timeNow - timeLast) * 0.001;

  // Calculate maximum thrust delta
  float thrustDelta = accLimit * 10 * timeDelta;  // maximum change in thrust component percentage in this loop [%]

  // Calculate actual thrust delta
  int q1Delta = q1Setpoint - q1Last;
  int q2Delta = q2Setpoint - q2Last;
  int q3Delta = q3Setpoint - q3Last;
  int q4Delta = q4Setpoint - q4Last;
  
  // Limit Q1 output thrust if change in setpoint is greater than allowable delta 
  if (abs(q1Delta) > thrustDelta) {
    if (q1Delta > 0) {
    q1Out = q1Last + thrustDelta;
    }
    else {
    q1Out = q1Last - thrustDelta;
    }
  }
  else {
    q1Out = q1Setpoint;
  }

  // Limit Q2 output thrust if change in setpoint is greater than allowable delta 
  if (abs(q2Delta) > thrustDelta) {
    if (q2Delta > 0) {
    q2Out = q2Last + thrustDelta;
    }
    else {
    q2Out = q2Last - thrustDelta;
    }
  }
  else {
    q2Out = q2Setpoint;
  }

  // Limit Q3 output thrust if change in setpoint is greater than allowable delta 
  if (abs(q3Delta) > thrustDelta) {
    if (q3Delta > 0) {
    q3Out = q3Last + thrustDelta;
    }
    else {
    q3Out = q3Last - thrustDelta;
    }
  }
  else {
    q3Out = q3Setpoint;
  }

  // Limit Q4 output thrust if change in setpoint is greater than allowable delta 
  if (abs(q4Delta) > thrustDelta) {
    if (q4Delta > 0) {
    q4Out = q4Last + thrustDelta;
    }
    else {
    q4Out = q4Last - thrustDelta;
    }
  }
  else {
    q4Out = q4Setpoint;
  }

  // Save timeNow and thruster output components for calculation in next loop
  timeLast = timeNow;
  q1Last = q1Out;
  q2Last = q2Out;
  q3Last = q3Out;
  q4Last = q4Out;
  
}

// Scale output to thrusters by a multiplier (typically to limit voltage output to thruster)
void scaleOutput() {

  q1Out = q1Out*voltMult;
  q2Out = q2Out*voltMult;
  q3Out = q3Out*voltMult;
  q4Out = q4Out*voltMult;

}

// Write output thruster values to motor controllers
void output2Thruster() {
  
  // reverse = 31, neutral = 47, forward = 63  
  int q1Bit = map(q1Out,-1000,1000,31,63);
  int q2Bit = map(q2Out,-1000,1000,31,63);
  int q3Bit = map(q3Out,-1000,1000,31,63);
  int q4Bit = map(q4Out,-1000,1000,31,63);

  // Write to motor controller pins
  analogWrite(q1Pin,q1Bit);
  analogWrite(q2Pin,q2Bit);
  analogWrite(q3Pin,q3Bit);
  analogWrite(q4Pin,q4Bit);
  
}

// Changes color of light
void changeLight(int lightColor) {
 
  // light color variable (0 = off, 1 = white, 2 = red, 3 = yellow, 4 = blue)

  if (lightColor == 2) {
    digitalWrite(redPin,LOW);
    digitalWrite(bluePin,HIGH);
    digitalWrite(greenPin,HIGH);
    Serial.println("red");
  }
  else if (lightColor == 3) {
    digitalWrite(redPin,LOW);
    digitalWrite(bluePin,HIGH);
    digitalWrite(greenPin,LOW);
    Serial.println("yellow");
  }
  
}
