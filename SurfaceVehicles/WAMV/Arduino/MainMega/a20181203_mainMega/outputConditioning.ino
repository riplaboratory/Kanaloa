void setpoint2output() {

  // Function code variables
  const byte accLimit = 200;     // acceleration limit (maximum change in thrust percentage*10 per second) [%/s]

  // Calculate time delta since last loop
  timeNow = millis();
  float timeDelta = (timeNow-timeLast)*0.001;

  // Calculate maximum thrust delta
  float thrustDelta = accLimit*timeDelta;   // maximum change in thrust component percentage in this loop [%]

  // Calculate actual thrust delta
  int Q1Delta = Q1Setpoint-Q1Last;
  int Q2Delta = Q2Setpoint-Q2Last;
  int Q3Delta = Q3Setpoint-Q3Last;
  int Q4Delta = Q4Setpoint-Q4Last;
  
  // Limit Q1 output thrust if change in setpoint is greater than allowable delta 
  if (abs(Q1Delta) > thrustDelta) {
    if (Q1Delta > 0) {
    Q1Out = Q1Last+thrustDelta;
    }
    else {
    Q1Out = Q1Last-thrustDelta;
    }
  }
  else {
    Q1Out = Q1Setpoint;
  }

  // Limit Q2 output thrust if change in setpoint is greater than allowable delta 
  if (abs(Q2Delta) > thrustDelta) {
    if (Q2Delta > 0) {
    Q2Out = Q2Last+thrustDelta;
    }
    else {
    Q2Out = Q2Last-thrustDelta;
    }
  }
  else {
    Q2Out = Q2Setpoint;
  }

  // Limit Q3 output thrust if change in setpoint is greater than allowable delta 
  if (abs(Q3Delta) > thrustDelta) {
    if (Q3Delta > 0) {
    Q3Out = Q3Last+thrustDelta;
    }
    else {
    Q3Out = Q3Last-thrustDelta;
    }
  }
  else {
    Q3Out = Q3Setpoint;
  }

  // Limit Q4 output thrust if change in setpoint is greater than allowable delta 
  if (abs(Q4Delta) > thrustDelta) {
    if (Q4Delta > 0) {
    Q4Out = Q4Last+thrustDelta;
    }
    else {
    Q4Out = Q4Last-thrustDelta;
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
  Q1Pub.publish(&Q1Msg);
  Q2Pub.publish(&Q2Msg);
  Q3Pub.publish(&Q3Msg);
  Q4Pub.publish(&Q4Msg);
  
}


void output2Pwm() {

  // Map thruster output (-1000 to 1000) to PWM bits (0 to 255)
  Q1BitOut = map(Q1Out,-1000,1000,-255,255);
  Q2BitOut = map(Q2Out,-1000,1000,-255,255);
  Q3BitOut = map(Q3Out,-1000,1000,-255,255);
  Q4BitOut = map(Q4Out,-1000,1000,-255,255);

  int Q1BitOutAbs = abs(round(Q1BitOut));
  int Q2BitOutAbs = abs(round(Q2BitOut));
  int Q3BitOutAbs = abs(round(Q3BitOut));
  int Q4BitOutAbs = abs(round(Q4BitOut));

  // Write PWM to pins
  analogWrite(pwmQ1Pin,Q1BitOutAbs);
  analogWrite(pwmQ2Pin,Q2BitOutAbs);
  analogWrite(pwmQ3Pin,Q3BitOutAbs);
  analogWrite(pwmQ4Pin,Q4BitOutAbs);
 
}

void output2RevCon(){
  
  // Control direction of reversing contactor
  if (Q1Out >= 0) {
    digitalWrite(revConQ1Pin,LOW); 
//    Serial.println("Q1 FORWARD");
  }
  else {
    digitalWrite(revConQ1Pin,HIGH);
//    Serial.printlng("Q1 REVERSE");
  } 
  if (Q2Out >= 0) {
    digitalWrite(revConQ2Pin,LOW);
//    Serial.println("Q2 FORWARD");
  }
  else {
    digitalWrite(revConQ2Pin,HIGH);
//    Serial.println("Q2 REVERSE");
  }
  if (Q3Out >= 0) {
    digitalWrite(revConQ3Pin,LOW);
//    Serial.println("Q3 FORWARD");
  }
  else {
    digitalWrite(revConQ3Pin,HIGH);
//    Serial.println("Q3 REVERSE");
  }
  if (Q4Out >= 0) {
    digitalWrite(revConQ4Pin,LOW);
//    Serial.println("Q4 FORWARD");
  }
  else {
    digitalWrite(revConQ4Pin,HIGH);
//    Serial.println("Q4 REVERSE");
    }

}
