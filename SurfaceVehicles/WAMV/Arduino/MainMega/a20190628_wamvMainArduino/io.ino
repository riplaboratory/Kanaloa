// Read in information from the kill Arduino, and assign mode of operation accordingly.  Note that the mainMega cannot actually kill the system, since the actual kill is handled by the killMega. 
void readKillArduino() {

  if (digitalRead(killCommPin) == LOW) {
    killStatus = 0;       // killMega sending LOW (UNKILL)
  }
  else {
    killStatus = 1;       // killMega sending HIGH (KILL)
  }
  if (digitalRead(revConKillCommPin) == LOW) {
    revConKillStatus = 0; // killMega reverse contactor sending LOW (UNKILL)
  }
  else {
    revConKillStatus = 1; // killMega reverse contactor sending HIGH (KILL)
  }
  if (digitalRead(modeCommPin) == LOW){
    modeStatus = 0;       // killMega auto/manual mode sending LOW (autonomous)
  }
  else {
    modeStatus = 1;       // killMega auto/manual mode sending HIGH (manual)
  }
  
}

// Filter noise from remote control joystick inputs using meadian of multiple measurements
void filterJoy() {

  // Preallocate arrays
  int ch1PulseArray[nMedian];
  int ch4PulseArray[nMedian];
  int ch6PulseArray[nMedian];

  // Calculate delay for each reading based on refresh rate of receiver
  int delayTime = (round((1/receiverRR)*1000));  // [ms]

  // Loop, taking a measurement each time and saving to array
  for (int i = 0; i <= nMedian - 1; i++) {

    // Take a measurement from the interrupt global variable
    ch1PulseArray[i] = ch1PulseRaw;   // yaw (right stick left-right)
    ch4PulseArray[i] = ch4PulseRaw;   // sway (left stick left-right)
    ch6PulseArray[i] = ch6PulseRaw;   // surge (left stick up-down)

    // Wait for receiver to push another value
    delay(delayTime);

  }

  // Save the median of each measurement array to global variable
  ch1Filtered = QuickMedian<int>::GetMedian(ch1PulseArray,nMedian);
  ch4Filtered = QuickMedian<int>::GetMedian(ch4PulseArray,nMedian);
  ch6Filtered = QuickMedian<int>::GetMedian(ch6PulseArray,nMedian);

  Serial.print("CH 1 (");
  Serial.print(ch1Filtered);
  Serial.print(")  CH 4 (");
  Serial.print(ch4Filtered);
  Serial.print(")  CH 6 (");
  Serial.print(ch6Filtered);
  Serial.println(")");

}

// Take joystick readings, and convert to setpoint thrust values
void joy2Setpoint() {

  // Grab ch1, ch4, and ch6 from filtering function
  int ch1Pulse = ch1Filtered;   // yaw (right stick left-right)
  int ch4Pulse = ch4Filtered;   // sway (left stick left-right)
  int ch6Pulse = ch6Filtered;   // surge (left stick up-down)

  // Pulse width variables
  int ch1PulseNeutral = round((ch1PulseMax - ch1PulseMin) / 2) + ch1PulseMin;
  int ch4PulseNeutral = round((ch4PulseMax - ch4PulseMin) / 2) + ch4PulseMin;
  int ch6PulseNeutral = round((ch6PulseMax - ch6PulseMin) / 2) + ch6PulseMin;

  // Remove the deadzone
  if (ch1Pulse < ch1PulseNeutral + round(pulseDeadzone/2) && ch1Pulse > ch1PulseNeutral - round(pulseDeadzone/2)) {
    ch1Pulse = ch1PulseNeutral;
  }
  if (ch4Pulse < ch4PulseNeutral + round(pulseDeadzone/2) && ch4Pulse > ch4PulseNeutral - round(pulseDeadzone/2)) {
    ch4Pulse = ch4PulseNeutral;
  }
  if (ch6Pulse < ch6PulseNeutral + round(pulseDeadzone/2) && ch6Pulse > ch6PulseNeutral - round(pulseDeadzone/2)) {
    ch6Pulse = ch6PulseNeutral;
  }

  // Map joystick inputs from -1000 to 1000
  int ch1Map = map(ch1Pulse,ch1PulseMin,ch1PulseMax,-1000,1000);
  int ch4Map = map(ch4Pulse,ch4PulseMin,ch4PulseMax,-1000,1000);
  int ch6Map = map(ch6Pulse,ch6PulseMin,ch6PulseMax,-1000,1000);

  // Calculate surge sway and yaw components
  int surgeQ1 = ch6Map;   // surge (forward positive)
  int surgeQ2 = ch6Map;
  int surgeQ3 = ch6Map;
  int surgeQ4 = ch6Map;
  int swayQ1 = -ch4Map;   // sway (right positive)
  int swayQ2 = ch4Map;
  int swayQ3 = -ch4Map;
  int swayQ4 = ch4Map;
  int yawQ1 = -ch1Map;    // yaw (CLOCKWISE positive)
  int yawQ2 = ch1Map;
  int yawQ3 = ch1Map;
  int yawQ4 = -ch1Map;

  // Map thruster components from -1000 to 1000
  q1Setpoint = constrain(surgeQ1 + swayQ1 + yawQ1,-1000,1000);
  q2Setpoint = constrain(surgeQ2 + swayQ2 + yawQ2,-1000,1000);
  q3Setpoint = constrain(surgeQ3 + swayQ3 + yawQ3,-1000,1000);
  q4Setpoint = constrain(surgeQ4 + swayQ4 + yawQ4,-1000,1000);

//  Serial.print("Q1: ");
//  Serial.print(q1Setpoint);
//  Serial.print("   Q2: ");
//  Serial.print(q2Setpoint);
//  Serial.print("   Q3: ");
//  Serial.print(q3Setpoint);
//  Serial.print("   Q4: ");
//  Serial.println(q4Setpoint);
  
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

  // Scale output to thruster by a multiplied (typically to limit voltage output to thruster)
  q1Out = q1Out*voltMult;
  q2Out = q2Out*voltMult;
  q3Out = q3Out*voltMult;
  q4Out = q4Out*voltMult;

//  Serial.print("Q1 (");
//  Serial.print(q1Out);
//  Serial.print(");  Q2 (");
//  Serial.print(q2Out);
//  Serial.print(");  Q3 (");
//  Serial.print(q3Out);
//  Serial.print(");  Q4 (");
//  Serial.print(q4Out);
//  Serial.println(");");
  
  // Publish to ROS
  q1Msg.data = q1Out;
  q2Msg.data = q2Out;
  q3Msg.data = q3Out;
  q4Msg.data = q4Out;
  q1Pub.publish(&q1Msg);
  q2Pub.publish(&q2Msg);
  q3Pub.publish(&q3Msg);
  q4Pub.publish(&q4Msg);

  // Output to thrusters
  const float servoRes = 4095;
  float q1ServoOut = abs(q1Out * servoRes / 1000);  // calculate the bit output to Q1 DAC
  pwm.setPWM(servoQ1Pin,0,q1ServoOut);            // set q1 bit command to servo shield
  float q2ServoOut = abs(q2Out * servoRes / 1000);  // calculate the bit output to Q2 DAC
  pwm.setPWM(servoQ2Pin,0,q2ServoOut);            // set q2 bit command to servo shield
  float q3ServoOut = abs(q3Out * servoRes / 1000);  // calculate the bit output to Q3 DAC
  pwm.setPWM(servoQ3Pin,0,q3ServoOut);            // set q3 bit command to servo shield
  float q4ServoOut = abs(q4Out * servoRes / 1000);  // calculate the bit output to Q4 DAC
  pwm.setPWM(servoQ4Pin,0,q4ServoOut);            // set q4 bit command to servo shield
  
}

// Calculate reverse contactor states and send to reverse contactors
void output2RevCon() {
  
  // Control direction of reversing contactor
  if (q1Out >= 0) {
    digitalWrite(revConQ1Pin,LOW);
  }
  else {
    digitalWrite(revConQ1Pin,HIGH);
  } 
  if (q2Out >= 0) {
    digitalWrite(revConQ2Pin,LOW);
  }
  else {
    digitalWrite(revConQ2Pin,HIGH);
  }
  if (q3Out >= 0) {
    digitalWrite(revConQ3Pin,LOW);
  }
  else {
    digitalWrite(revConQ3Pin,HIGH);
  }
  if (q4Out >= 0) {
    digitalWrite(revConQ4Pin,LOW);
  }
  else {
    digitalWrite(revConQ4Pin,HIGH);
  }  
}
