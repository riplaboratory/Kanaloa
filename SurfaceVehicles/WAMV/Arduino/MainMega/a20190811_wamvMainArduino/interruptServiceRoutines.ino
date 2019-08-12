/* Interrupt Service Routines are functions that execute in response to a system interrupt. 
 * These functions have some unique limitations most other functions to not have.  You can read more here:  
 *  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 *  
 *  If using external interrupts, the pins available on the MEGA 2560 are 2,3,18,19 (also technically 20, and 21, but they are very difficult to enable because these pins are shared with I2C).
 *  If using pin change interrupts, you must know the  the pins available on the MEGA 2560 are A8,A9,A10,A11,A12,A13,A14,A15
*/

void ch1Rise() {

  // Attach interrupt to respective fall function
  attachInterrupt(digitalPinToInterrupt(ch1Pin),ch1Fall,FALLING);

  // Start timer
  ch1Timer = micros();
  
}

void ch1Fall() {

  // Attach interrupt to respective rise function
  attachInterrupt(digitalPinToInterrupt(ch1Pin),ch1Rise,RISING);
  
  // Save reading to local variable
  int reading = micros() - ch1Timer;

  // Only commit reading to global variable if it's within expected range
  if (reading >= ch1PulseMin - pulseTolerance && reading <= ch1PulseMax + pulseTolerance) { 
    ch1PulseRaw = reading;
  }
  
}

void ch4Rise() {
  
  // Attach interrupt to respective fall function
  attachInterrupt(digitalPinToInterrupt(ch4Pin),ch4Fall,FALLING);

  // Start timer
  ch4Timer = micros();
  
}

void ch4Fall() {

  // Attach interrupt to respective rise function
  attachInterrupt(digitalPinToInterrupt(ch4Pin),ch4Rise,RISING);
  
  // Save reading to local variable
  int reading = micros() - ch4Timer;

  // Only commit reading to global variable if it's within expected range
  if (reading >= ch4PulseMin - pulseTolerance && reading <= ch4PulseMax + pulseTolerance) { 
    ch4PulseRaw = reading;
  }
  
}

void ch6Rise() {
  
  // Attach interrupt to respective fall function
  attachInterrupt(digitalPinToInterrupt(ch6Pin),ch6Fall,FALLING);

  // Start timer
  ch6Timer = micros();
  
}

void ch6Fall() {

  // Attach interrupt to respective rise function
  attachInterrupt(digitalPinToInterrupt(ch6Pin),ch6Rise,RISING);
  
  // Save reading to local variable
  int reading = micros() - ch6Timer;

  // Only commit reading to global variable if it's within expected range
  if (reading >= ch6PulseMin - pulseTolerance && reading <= ch6PulseMax + pulseTolerance) { 
    ch6PulseRaw = reading;
  }
  
}
