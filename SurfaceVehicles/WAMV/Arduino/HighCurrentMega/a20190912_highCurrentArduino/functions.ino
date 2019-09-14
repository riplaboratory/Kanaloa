void requestFromSlave(){
  // Function to request motor commands to send to motors from
  // Arduino in low current box. 

  int i = 0;
  Wire.requestFrom(SLAVE_ADDR, ANSWERSIZE); // request numOfBits (30) from SLAVE_ADDR (9)
  
  while(Wire.available()>0){
    char c = Wire.read();  // receive bit as a character
    message[i] = c;
    i++;
  }
}

void parseMotorCmds(){
    // Get all the motor commands as a string from the I2C message array
  for(int j=2; j<6; j++){
    q1Msg += message[j];    // Since the motor command message comes in
    q2Msg += message[j+7];  // from the master as one long string with 
    q3Msg += message[j+14]; // all the commands in one line, we can grab
    q4Msg += message[j+21]; // all of them in parallel as Strings.
  }

  // Convert all the motor commands from Strings to Integers
  q1Motor = q1Msg.toInt(); 
  q2Motor = q2Msg.toInt();
  q3Motor = q3Msg.toInt();
  q4Motor = q4Msg.toInt();

  // Reset the motor command String messages
  q1Msg = "";
  q2Msg = "";
  q3Msg = "";
  q4Msg = "";

  // Get the motor spin directions
  q1Dir = message[6];
  q2Dir = message[13];
  q3Dir = message[20];
  q4Dir = message[27];

  // For debugging purposes, prints the motor command messages
  if(debug == true){
    Serial.print("q1Motor: ");
    Serial.println(q1Motor);
    Serial.print("q2Motor: ");
    Serial.println(q2Motor);
    Serial.print("q3Motor: ");
    Serial.println(q3Motor);
    Serial.print("q4Motor: ");
    Serial.println(q4Motor); 
    Serial.print("q1Dir: ");
    Serial.println(q1Dir);
    Serial.print("q2Dir: ");
    Serial.println(q2Dir);
    Serial.print("q3Dir: ");
    Serial.println(q3Dir);
    Serial.print("q4Dir: ");
    Serial.println(q4Dir); 
  }
}
