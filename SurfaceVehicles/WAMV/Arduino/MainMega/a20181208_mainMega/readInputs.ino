void readKill() {
  // Read digital in from killMega, and assign mode of operation accordingly.  Note that the mainMega cannot actually kill the system, since the actual kill is handled by the killMega.  Therefore, this information is purely provided to the mainMega to inform subsequent code execution.

  if (digitalRead(remKillPin) == HIGH) {
    // mainMega reads HIGH (unkill) signal from killMega
    remKill = 2;
  }
  else if (digitalRead(remKillPin) == LOW) {
    // mainMega reads LOW (kill) signal from killMega
    remKill = 1;
  }
  else {
    remKill = 0;
    Serial.println("Kill switch signal from killMega registering invalid input, please check wiring...");
  }
}

void readSwitches() {
  // Read PWM switch channels from receiver and assign appropriate modes of operation

  // Assign global variables to local function copy
  int tolerance = 50;
  int ch5Current = ch5PulseWidth;
//  int ch7Current = ch7PulseWidth;

  // Determine auto/manual (ch5) state
  if (ch5Current + tolerance > 2040 && ch5Current - tolerance < 2040) {
    // SF switch on transmitter is in the up position (autonomous)
    mode = 2;
    digitalWrite(statePin,LOW);     // set LOW for autonomous
  }
  else if (ch5Current + tolerance > 1016 && ch5Current - tolerance < 1016) {
    // SF switch on transmitter is in the down position (manual)
    mode = 1;
    digitalWrite(statePin,HIGH);    // set HIGH for manual
  }
  else {
    // Invalid input from ch5
    mode = 0;
    Serial.println("State switch (channel 5) reporting invalid input; check transmitter connection. KILLING...");
    digitalWrite(statePin,HIGH);    // set HIGH for manual
    digitalWrite(remKillPin,LOW);   // set remKillPin LOW to kill
  }

//  // Determines reversing contactor (ch7) state
//  if (ch7Current + tolerance > 1044 && ch7Current - tolerance < 1044) {
//    // SB switch on transmitter is in the up position (unkill reversing contactors)
//    revConKill = 2;
//    if (remKill == 2) {
//      digitalWrite(revConKillPin,HIGH);      // set HIGH to unkill
//    }
//    else {
//      digitalWrite(revConKillPin,LOW);     // kill switch is engaged anyway, might as well kill revCon as well
//    }
//  }
//  else if (ch7Current + tolerance > 1548 && ch7Current - tolerance < 1548) {
//    // SB switch on transmitter is in the middle position (kill reversing contactors)
//    revConKill = 1;
//    digitalWrite(revConKillPin,LOW);    // set LOW to kill
//  }
//  else if (ch7Current + tolerance > 2070 && ch7Current - tolerance < 2070) {
//    // SB switch on transmitter is in the down position (kill)
//    revConKill = 1;
//    digitalWrite(revConKillPin,LOW);    // set LOW to kill
//  }
//  else {
//    // Invalid input from ch7
//    revConKill = 0;
//    Serial.println("Reversing contactor kill switch (channel 7) reporting invalid input; check transmitter connection. KILLING...");
//    digitalWrite(revConKillPin,LOW);    // set LOW to kill
//    digitalWrite(remKillPin,HIGH);      // set remKillPin HIGH to kill
//  }
}

void readJoysticks() {
  // Read joystick channels from receiver and assign appropriate modes of operation

  // Assign global variables to local function copy
  int ch1Current = ch1PulseWidth;
  int ch4Current = ch4PulseWidth;
  int ch6Current = ch6PulseWidth;
  
  // Function code variables
  int deadZone = 200;          // +/- percentage of middle area of joysticks for zero input [%] (min = -100; max = 100) Originally 10

  // Calculate voltage multiplier based on battery voltage
  float voltMult = voltOutMax / voltMain;
  if (voltMult >= 1) {
    Serial.println("Serious problem encountered with main battery voltage calculation...disabling thruster output");
    voltMult = 0;
  }
  
  // Map receiver inputs from -100 to 100
  int yawIn = map(ch1Current,1000,2000,1000,-1000);     // yaw (positive = CCW, negative = CW)
  int surgeIn = map(ch6Current,1000,2000,-1000,1000);    // surge (positive = forward, negative = backward)
  int swayIn = map(ch4Current,1000,2000,-1000,1000);     // sway (positive = right, negative = left)

  // Apply dead zone to controller components
  if (yawIn <= deadZone && yawIn >= -deadZone) {
    yawIn = 0;
  }
  if (surgeIn <= deadZone && surgeIn >= -deadZone) {
    surgeIn = 0;
  }
  if (swayIn <= deadZone && swayIn >= -deadZone) {
    swayIn = 0;
  }
  
  // Calculate surge, sway, and yaw components
  int surgeQ1 = surgeIn;
  int surgeQ2 = surgeIn;
  int surgeQ3 = surgeIn;
  int surgeQ4 = surgeIn;
  int swayQ1 = -swayIn;
  int swayQ2 = swayIn;
  int swayQ3 = -swayIn;
  int swayQ4 = swayIn;
  int yawQ1 = yawIn;
  int yawQ2 = -yawIn;
  int yawQ3 = -yawIn;
  int yawQ4 = yawIn;

  // Sum surge, sway, and yaw components, and constrain to range -1000 to 1000
  Q1Setpoint = constrain(surgeQ1+swayQ1+yawQ1,-1000,1000);
  Q2Setpoint = constrain(surgeQ2+swayQ2+yawQ2,-1000,1000);
  Q3Setpoint = constrain(surgeQ3+swayQ3+yawQ3,-1000,1000);
  Q4Setpoint = constrain(surgeQ4+swayQ4+yawQ4,-1000,1000);

  // Scale setpoint components based on battery voltage multiplier
  Q1Setpoint = Q1Setpoint*voltMult;
  Q2Setpoint = Q2Setpoint*voltMult;
  Q3Setpoint = Q3Setpoint*voltMult;
  Q4Setpoint = Q4Setpoint*voltMult;

//  Serial.print("Q1: ");
//  Serial.print(Q1Setpoint);
//  Serial.print("   Q2: ");
//  Serial.print(Q2Setpoint);
//  Serial.print("   Q3: ");
//  Serial.print(Q3Setpoint);
//  Serial.print("   Q4: ");
//  Serial.println(Q4Setpoint);

}
