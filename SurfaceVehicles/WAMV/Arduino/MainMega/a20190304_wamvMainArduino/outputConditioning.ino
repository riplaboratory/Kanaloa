// Calculate acceleration-limited outputs as a function of the setpoints.
void setpoint2Output() {

  // Function code variables
  const byte accLimit = 75;     // acceleration limit (maximum change in thrust percentage per second) [%/s] Originally 50

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
  float q1ServoOut = abs(q1Out * servoRes / 100);   // calculate the bit output to Q1 DAC
  pwm.setPWM(servoQ1Pin,0,q1ServoOut);              // set q1 bit command to servo shield
  float q2ServoOut = abs(q2Out * servoRes / 100);   // calculate the bit output to Q2 DAC
  pwm.setPWM(servoQ2Pin,0,q2ServoOut);              // set q2 bit command to servo shield
  float q3ServoOut = abs(q3Out * servoRes / 100);   // calculate the bit output to Q3 DAC
  pwm.setPWM(servoQ3Pin,0,q3ServoOut);              // set q3 bit command to servo shield
  float q4ServoOut = abs(q4Out * servoRes / 100);   // calculate the bit output to Q4 DAC
  pwm.setPWM(servoQ4Pin,0,q4ServoOut);              // set q4 bit command to servo shield
  
}

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
