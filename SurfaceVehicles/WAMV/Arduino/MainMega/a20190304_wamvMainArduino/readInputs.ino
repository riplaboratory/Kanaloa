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

void joy2Setpoint() {

  // Grab ch1, ch4, and ch6 from interrupt global variables
  int ch1Pulse = ch1PulseWidth;   // yaw (right stick left-right)
  int ch4Pulse = ch4PulseWidth;   // sway (left stick left-right)
  int ch6Pulse = ch6PulseWidth;   // surge (left stick up-down)

  // Pulse width variables
  int ch1PulseMin = 1200;
  int ch1PulseMax = 2170;
  int ch1PulseNeutral = round((ch1PulseMax - ch1PulseMin) / 2) + ch1PulseMin;
  int ch4PulseMin = 1160;
  int ch4PulseMax = 2160;
  int ch4PulseNeutral = round((ch4PulseMax - ch4PulseMin) / 2) + ch4PulseMin;
  int ch6PulseMin = 1240;
  int ch6PulseMax = 2240;
  int ch6PulseNeutral = round((ch6PulseMax - ch6PulseMin) / 2) + ch6PulseMin;
  int pulseDeadzone = 100;

  // Remove the deadzone
  if (ch1PulseWidth < ch1PulseNeutral + round(pulseDeadzone/2) && ch1PulseWidth > ch1PulseNeutral - round(pulseDeadzone/2)) {
    ch1PulseWidth = ch1PulseNeutral;
  }
  if (ch4PulseWidth < ch4PulseNeutral + round(pulseDeadzone/2) && ch4PulseWidth > ch4PulseNeutral - round(pulseDeadzone/2)) {
    ch4PulseWidth = ch4PulseNeutral;
  }
  if (ch6PulseWidth < ch6PulseNeutral + round(pulseDeadzone/2) && ch6PulseWidth > ch6PulseNeutral - round(pulseDeadzone/2)) {
    ch6PulseWidth = ch6PulseNeutral;
  }

  // Map joystick inputs from -1000 to 1000
  int ch1Map = map(ch1PulseWidth,ch1PulseMin,ch1PulseMax,-1000,1000);
  int ch4Map = map(ch4PulseWidth,ch4PulseMin,ch4PulseMax,-1000,1000);
  int ch6Map = map(ch6PulseWidth,ch6PulseMin,ch6PulseMax,-1000,1000);

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

