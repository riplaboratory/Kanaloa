// --------- FUNCTIONS -----------------
String getVoltageMsg(float voltage){
  String msg;
  
  if (abs(voltage) < 10){
    msg = "0" + String(voltage);
  }
  else{
    msg = String(voltage);
  }
  return msg;
}

void msgReset(){
  /***
   * Function to reset the serial messages after each loop. 
   */
  voltageMsg = "V";
}

void requestFromSlave(){
  // Function to request motor commands to send to motors from
  // Arduino in low current box. 

  Wire.requestFrom(SLAVE_ADDR, ANSWERSIZE); // request number of bytes (14) from SLAVE_ADDR (9)

  // Save received message to array
  for (int i=0; i<ANSWERSIZE; i++){
    char c = Wire.read();
    message[i] = c;
  }
}

void sendVoltageMsg(){
  /***
   * Function to send serial messages for all four motor commands.
   * It iterates over the serial message Strings because Serial.write
   * cannot take Strings as input.
   */
   
  voltageMsg += getVoltageMsg(voltage);
  
  Wire.beginTransmission(SLAVE_ADDR);
  // Send voltage measurement
  for(int i=0; i<=4; i++){
    Wire.write(voltageMsg[i]);
  }
  Wire.endTransmission();
  msgReset();
  
}

void parseMotorCmds(){
  // Get all the motor commands as a string from the I2C message array
  for(int j=0; j<4; j++){
    cQ1Motor[j] = message[j+2];
    cQ2Motor[j] = message[j+9];
  }

  // Convert all the motor commands from C-strings to integers
  q1Motor = atoi(cQ1Motor);
  q2Motor = atoi(cQ2Motor);

  // Get motor directions
  q1Dir = message[6];
  q2Dir = message[13];

  // Make motors negative if direction is reverse
  if (q1Dir == 'R'){
    q1Motor = -q1Motor; 
  }
  else if (q1Dir == 'N'){
    q1Motor = 0; 
  }

  if (q2Dir == 'R'){
    q2Motor = -q2Motor;
  }
  else if (q2Dir == 'N'){
    q2Motor = 0; 
  }

//  // Voltage setpoints
//  Serial.print(q1.getVoltageSetpoint());
//  Serial.print(",");
//  Serial.print(q2.getVoltageSetpoint());
//  Serial.print(",");
//  Serial.print(q3.getVoltageSetpoint());
//  Serial.print(",");
//  Serial.print(q4.getVoltageSetpoint());
//  Serial.println();

  // Actual motor voltages (i.e. voltage feedback)
//  Serial.print("Q1:"); Serial.print(q1.getMotorVoltage());
//  Serial.print(" "); 
//  Serial.print("Q2:"); Serial.print(q2.getMotorVoltage());
//  Serial.print(" ");
//  Serial.print("Q3:"); Serial.print(q3.getMotorVoltage());
//  Serial.print(" "); 
//  Serial.print("Q4:"); Serial.println(q4.getMotorVoltage());

  // MC1 is GOOD
  // MC2 is sort of reading what it should but its turning on and off for some reason
  // MC3 only reads 2.4
  // MC4 is OK (sort of goes on and off, but sort of like MC1).


  // Reset the motor command C-strings
  for (int i=0; i<4; i++){
    cQ1Motor[i] = '0';
    cQ2Motor[i] = '0';
  }

  // For debugging purposes, prints the motor command messages
  if(debug == true){
//    Serial.print("q1Motor: "); Serial.println(q1Motor);
//    Serial.print("q2Motor: "); Serial.println(q2Motor);
//    Serial.print("q3Motor: "); Serial.println(q3Motor);
//    Serial.print("q4Motor: "); Serial.println(q4Motor); 
//    Serial.print("q1Dir: "); Serial.println(q1Dir);
//    Serial.print("q2Dir: "); Serial.println(q2Dir);
//    Serial.print("q3Dir: "); Serial.println(q3Dir);
//    Serial.print("q4Dir: "); Serial.println(q4Dir); 
//
//    Serial.print("V_set: ("); 
//    Serial.print(q1.getVoltageSetpoint);Serial.print(", ");
//    Serial.print(q2.getVoltageSetpoint);Serial.print(", ");
//    Serial.print(q3.getVoltageSetpoint);Serial.print(", ");
//    Serial.print(q4.getVoltageSetpoint);Serial.println(")");
//    Serial.print("V_measured: ("); 
//    Serial.print(q1.getMotorVoltage);Serial.print(", ");
//    Serial.print(q2.getMotorVoltage);Serial.print(", ");
//    Serial.print(q3.getMotorVoltage);Serial.print(", ");
//    Serial.print(q4.getMotorVoltage);Serial.print(")");
  }
}

void getVoltage(){
  sensorValue = analogRead(voltagePin);
  voltage = 0.0302 * sensorValue + 0.0258;
  
//  Serial.print("Main Battery: "); Serial.println(voltage);
}
