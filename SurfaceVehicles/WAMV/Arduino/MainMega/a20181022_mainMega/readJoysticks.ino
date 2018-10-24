void joy2setpoint() {

  // Function code variables
  unsigned int deadZone = 20;     // +/- percentage of middle area of joysticks for zero input [%] (min = -100; max = 100) Originally 10

  // Read signals from transmitter and encode it for motors
  unsigned int ch1Raw = pulseIn(ch1Pin, HIGH); // PWM in from remote control receiver channel 1 (right stick left-to-right) for yaw control. left = 1080, right = 1891.
  unsigned int ch6Raw = pulseIn(ch6Pin, HIGH); // PWM in from remote control receiver channel 6 (left stick up-to-down) for surge control. up = 1075, right = 1900.
  unsigned int ch4Raw = pulseIn(ch4Pin, HIGH); // PWM in from remote control receiver channel 4 (left stick left-to-right) for sway control. left = 1075, right = 1900.

  // Calculate voltage multiplier based on battery voltage
  float voltMult = voltOutMax / voltMain;
  Serial.print("ch1Raw (right stick left-to-right): ");
  Serial.println(ch1Raw);
  Serial.print("ch6Raw (left stick up-to-down): ");
  Serial.println(ch6Raw);
  Serial.print("ch4Raw (left stick up-to-down): ");
  Serial.println(ch4Raw);
  if (voltMult >= 1) {
    Serial.println("Serious problem encountered with main battery voltage calculation...disabling thruster output");
    voltMult = 0;
  }
  
  // Map receiver inputs from -100 to 100
  int yawIn = map(ch1Raw, pulseMin, pulseMax, 100, -100); // yaw (positive = CCW, negative = CW)
  int surgeIn = map(ch6Raw, pulseMin, pulseMax, -100, 100); // surge (positive = forward, negative = backward)
  int swayIn = map(ch4Raw, pulseMin, pulseMax, -100, 100); // sway (positive = right, negative = left)

  // Apply dead zone to controller components
  if (yawIn <= deadZone || yawIn >= -deadZone) {
    yawIn = 0;
  }
  if (surgeIn <= deadZone || surgeIn >= -deadZone) {
    surgeIn = 0;
  }
  if (swayIn <= deadZone || swayIn >= -deadZone) {
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

  // Sum surge, sway, and yaw components
  Q1Setpoint = surgeQ1 + swayQ1 + yawQ1;
  Q2Setpoint = surgeQ2 + swayQ2 + yawQ2;
  Q3Setpoint = surgeQ3 + swayQ3 + yawQ3;
  Q4Setpoint = surgeQ4 + swayQ4 + yawQ4;

  // Limit setpoint components to no less than -100 and no more than 100
  Q1Setpoint = constrain(Q1Setpoint, -100, 100);
  Q2Setpoint = constrain(Q2Setpoint, -100, 100);
  Q3Setpoint = constrain(Q3Setpoint, -100, 100);
  Q4Setpoint = constrain(Q4Setpoint, -100, 100);

  // Scale setpoint components based on battery voltage
  Q1Setpoint = Q1Setpoint * voltMult;
  Q2Setpoint = Q2Setpoint * voltMult;
  Q3Setpoint = Q3Setpoint * voltMult;
  Q4Setpoint = Q4Setpoint * voltMult;
}

void setpoint2output() {

  // Function code variables
  const byte accLimit = 40;     // acceleration limit (maximum change in thrust percentage per second) [%/s] Originally 50

  // Calculate time delta since last loop
  timeNow = millis();
  float timeDelta = (timeNow - timeLast) * 0.001;

  // Calculate maximum thrust delta
  float thrustDelta = accLimit * timeDelta;   // maximum change in thrust component percentage in this loop [%]

  // Calculate actual thrust delta
  int Q1Delta = Q1Setpoint - Q1Last;
  int Q2Delta = Q2Setpoint - Q2Last;
  int Q3Delta = Q3Setpoint - Q3Last;
  int Q4Delta = Q4Setpoint - Q4Last;
  
  // Limit Q1 output thrust if change in setpoint is greater than allowable delta 
  if (abs(Q1Delta) > thrustDelta) {
    if (Q1Delta > 0) {
    Q1Out = Q1Last + thrustDelta;
    }
    else {
    Q1Out = Q1Last - thrustDelta;
    }
  }
  else {
    Q1Out = Q1Setpoint;
  }

  // Limit Q2 output thrust if change in setpoint is greater than allowable delta 
  if (abs(Q2Delta) > thrustDelta) {
    if (Q2Delta > 0) {
    Q2Out = Q2Last + thrustDelta;
    }
    else {
    Q2Out = Q2Last - thrustDelta;
    }
  }
  else {
    Q2Out = Q2Setpoint;
  }

  // Limit Q3 output thrust if change in setpoint is greater than allowable delta 
  if (abs(Q3Delta) > thrustDelta) {
    if (Q3Delta > 0) {
    Q3Out = Q3Last + thrustDelta;
    }
    else {
    Q3Out = Q3Last - thrustDelta;
    }
  }
  else {
    Q3Out = Q3Setpoint;
  }

  // Limit Q4 output thrust if change in setpoint is greater than allowable delta 
  if (abs(Q4Delta) > thrustDelta) {
    if (Q4Delta > 0) {
    Q4Out = Q4Last + thrustDelta;
    }
    else {
    Q4Out = Q4Last - thrustDelta;
    }
  }
  else {
    Q4Out = Q4Setpoint;
  }

  // Save timeNow and thruster output components for calculation in next loop
  timeLast = timeNow;
  Q1Last = Q1Out;
  Q2Last = Q2Out;
  Q3Last = Q3Out;
  Q4Last = Q4Out;

  // Publish to ROS
  Q1Msg.data = Q1Out;
  Q2Msg.data = Q2Out;
  Q3Msg.data = Q3Out;
  Q4Msg.data = Q4Out;
  Q1Pub.publish( &Q1Msg);
  Q2Pub.publish( &Q2Msg);
  Q3Pub.publish( &Q3Msg);
  Q4Pub.publish( &Q4Msg);
  
}


void output2dacRevCon() {
  
//  const byte Q1Port = 0;
//  const byte Q2Port = 3;
//  const byte Q3Port = 2;
//  const byte Q4Port = 3;
//  const int dacRes = 4095;
//  
//  //maxres (5v) is 4095
//  byte Q1Pin = 3;
//  byte Q2Pin = 2;
//  byte Q3Pin = 0;
//  byte Q4Pin = 1;
//
//  // Output analog signal to DACs
//  Q1DacOut = abs(Q1Out * dacRes / 100);     // calculate the bit output to Q1 DAC
//  pwm.setPWM(Q1Pin,0,Q1DacOut);          // Set the voltage to send to the motor
//  Q2DacOut = abs(Q2Out * dacRes / 100);     // calculate the bit output to Q2 DAC
//  pwm.setPWM(Q2Pin,0,Q2DacOut);          // Set the voltage to send to the motor
//  Q3DacOut = abs(Q3Out * dacRes / 100);     // calculate the bit output to Q3 DAC
//  pwm.setPWM(Q3Pin,0,Q3DacOut);          // Set the voltage to send to the motor
//  Q4DacOut = abs(Q4Out * dacRes / 100);     // calculate the bit output to Q4 DAC
//  pwm.setPWM(Q4Pin,0,Q4DacOut);          // Set the voltage to send to the motor

  // Map thruster output (-100 to 100) to PWM bits (0 to 255) (really shouldn't be called "DacOut")
  Q1DacOut = map(Q1Out,-100,100,-255,255);
  Q2DacOut = map(Q2Out,-100,100,-255,255);
  Q3DacOut = map(Q3Out,-100,100,-255,255);
  Q4DacOut = map(Q4Out,-100,100,-255,255);

  int Q1DacOutInt = abs(round(Q1DacOut));
  int Q2DacOutInt = abs(round(Q2DacOut));
  int Q3DacOutInt = abs(round(Q3DacOut));
  int Q4DacOutInt = abs(round(Q4DacOut));

  // Write PWM to pins
  analogWrite(pwmQ1Pin,Q1DacOutInt);
  analogWrite(pwmQ2Pin,Q2DacOutInt);
  analogWrite(pwmQ3Pin,Q3DacOutInt);
  analogWrite(pwmQ4Pin,Q4DacOutInt);

//  // Write PWM to pins
//  analogWrite(pwmQ1Pin,140);
//  analogWrite(pwmQ2Pin,140);
//  analogWrite(pwmQ3Pin,140);
//  analogWrite(pwmQ4Pin,140);

  // Control direction of reversing contactor
  if (Q1Out >= 0) {
    digitalWrite(revConQ1Pin,HIGH); 
//    Serial.println("Q1 FORWARD");
  }
  else {
    digitalWrite(revConQ1Pin,LOW);
//    Serial.printlng("Q1 REVERSE");
  } 
  if (Q2Out >= 0) {
    digitalWrite(revConQ2Pin,HIGH);
//    Serial.println("Q2 FORWARD");
  }
  else {
    digitalWrite(revConQ2Pin,LOW);
//    Serial.println("Q2 REVERSE");
  }
  if (Q3Out >= 0) {
    digitalWrite(revConQ3Pin,HIGH);
//    Serial.println("Q3 FORWARD");
  }
  else {
    digitalWrite(revConQ3Pin,LOW);
//    Serial.println("Q3 REVERSE");
  }
  if (Q4Out >= 0) {
    digitalWrite(revConQ4Pin,HIGH);
//    Serial.println("Q4 FORWARD");
  }
  else {
    digitalWrite(revConQ4Pin,LOW);
//    Serial.println("Q4 REVERSE");
    }
  
}
