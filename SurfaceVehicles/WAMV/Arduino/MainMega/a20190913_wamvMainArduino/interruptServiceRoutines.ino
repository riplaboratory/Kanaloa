/* Interrupt Service Routines are functions that execute in response to a system interrupt. 
 * These functions have some unique limitations most other functions to not have.  
 * 
 * Note that the Arduino Mega 2560 can use pin change interrupts on pins 10,11,12,13,50,51,52,53,A8(62),A9(63),A10(64),A11(65),A12(66),A13(67),A14(68),A15(69). 
*/

void ch1Change() {

  // Check the current state of the pin
  ch1State = digitalRead(ch1Pin);

  if (ch1State == 1) {
    // Pin is high, this was probably a rise event

    // Save current time to timer
    ch1Timer = micros();

  }

  else {
    // Pin is low, this was probably a fall event

    // Subtract ch1Time from current time
    ch1PulseRaw = micros() - ch1Timer;

    // Only commit reading to memory if it's within expected range
    if (ch1PulseRaw >= ch1PulseMin - pulseTolerance && ch1PulseRaw <= ch1PulseMax + pulseTolerance) {

      // Move new measurement into circular buffer array
      memcpy(ch1PulseArray, &ch1PulseArray[1], sizeof(ch1PulseArray) - sizeof(int));
      ch1PulseArray[nMedian - 1] = ch1PulseRaw;

    }

  }

}

void ch2Change() {

  // Check the current state of the pin
  ch2State = digitalRead(ch2Pin);

  if (ch2State == 1) {
    // Pin is high, this was probably a rise event

    // Save current time to timer
    ch2Timer = micros();

  }

  else {
    // Pin is low, this was probably a fall event

    // Subtract ch1Time from current time
    ch2PulseRaw = micros() - ch2Timer;

    // Only commit reading to global variable if it's within expected range
    if (ch2PulseRaw >= ch2PulseMin - pulseTolerance && ch2PulseRaw <= ch2PulseMax + pulseTolerance) {

      // Move new measurement into circular buffer array
      memcpy(ch2PulseArray, &ch2PulseArray[1], sizeof(ch2PulseArray) - sizeof(int));
      ch2PulseArray[nMedian - 1] = ch2PulseRaw;

    }

  }

}

void ch3Change() {

  // Check the current state of the pin
  ch3State = digitalRead(ch3Pin);

  if (ch3State == 1) {
    // Pin is high, this was probably a rise event

    // Save current time to timer
    ch3Timer = micros();

  }

  else {
    // Pin is low, this was probably a fall event

    // Subtract ch1Time from current time
    ch3PulseRaw = micros() - ch3Timer;

    // Only commit reading to global variable if it's within expected range
    if (ch3PulseRaw >= ch3PulseMin - pulseTolerance && ch3PulseRaw <= ch3PulseMax + pulseTolerance) {

      // Move new measurement into circular buffer array
      memcpy(ch3PulseArray, &ch3PulseArray[1], sizeof(ch3PulseArray) - sizeof(int));
      ch3PulseArray[nMedian - 1] = ch3PulseRaw;

    }

  }

}

void ch5Change() {

  // Check the current state of the pin
  ch5State = digitalRead(ch5Pin);

  if (ch5State == 1) {
    // Pin is high, this was probably a rise event

    // Save current time to timer
    ch5Timer = micros();

  }

  else {
    // Pin is low, this was probably a fall event

    // Subtract ch1Time from current time
    ch5PulseRaw = micros() - ch5Timer;

    // Only commit reading to global variable if it's within expected range
    if (ch5PulseRaw >= ch5PulseMin - pulseTolerance && ch5PulseRaw <= ch5PulseMax + pulseTolerance) {

      // Move new measurement into circular buffer array
      memcpy(ch5PulseArray, &ch5PulseArray[1], sizeof(ch5PulseArray) - sizeof(int));
      ch5PulseArray[nMedian - 1] = ch5PulseRaw;

    }

  }

} 

void ch6Change() {

  // Check the current state of the pin
  ch6State = digitalRead(ch6Pin);

  if (ch6State == 1) {
    // Pin is high, this was probably a rise event

    // Save current time to timer
    ch6Timer = micros();

  }

  else {
    // Pin is low, this was probably a fall event

    // Subtract ch1Time from current time
    ch6PulseRaw = micros() - ch6Timer;

    // Only commit reading to global variable if it's within expected range
    if (ch6PulseRaw >= ch6PulseMin - pulseTolerance && ch6PulseRaw <= ch6PulseMax + pulseTolerance) {

      // Move new measurement into circular buffer array
      memcpy(ch6PulseArray, &ch6PulseArray[1], sizeof(ch6PulseArray) - sizeof(int));
      ch6PulseArray[nMedian - 1] = ch6PulseRaw;

    }

  }

}

void sendMsgs(){
  /***
   * Function to send serial messages for motor commands.
   * It iterates over the serial message Strings because Serial.write
   * cannot take Strings as input.
   */
  byte response[ANSWERSIZE];

  for (byte i=0; i<ANSWERSIZE; i++){
    response[i] = (byte)motorCmds.charAt(i);
  }
  Wire.write(response, ANSWERSIZE);
}

void readVoltageMsg(int byteCount) {
  //Read while data received
  while(0 < Wire.available()) { 
    char c = Wire.read();     // Read byte as a character and save to variable c
    incoming[counter] = c;          // Append character to character array 
    counter++;                      // Increment i counter variable
  }
  counter = 0; // Reset the counter variable
}
