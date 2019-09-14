/* Interrupt Service Routines are functions that execute in response to a system interrupt. 
 * These functions have some unique limitations most other functions to not have.  You can read more here:  
 *  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 *  
 *  If using external interrupts, the pins available on the MEGA 2560 are 2,3,18,19 (also technically 20, and 21, but they are very difficult to enable because these pins are shared with I2C).
 *  If using pin change interrupts, you must know the  the pins available on the MEGA 2560 are A8,A9,A10,A11,A12,A13,A14,A15
*/

void ch5Rise() {

  // Attach interrupt to respective fall function
  attachInterrupt(digitalPinToInterrupt(ch5Pin),ch5Fall,FALLING);

  // Start timer
  ch5Timer = micros();
  
}

void ch5Fall() {

  // Attach interrupt to respective rise function
  attachInterrupt(digitalPinToInterrupt(ch5Pin),ch5Rise,RISING);
  
  // Save reading to local variable
  int reading = micros() - ch5Timer;

  // Only commit reading to global variable if it's within expected range
  if (reading >= ch5PulseMin - pulseTolerance && reading <= ch5PulseMax + pulseTolerance) { 
    ch5PulseRaw = reading;
  }
  
}

void ch7Rise() {
  
  // Attach interrupt to respective fall function
  attachInterrupt(digitalPinToInterrupt(ch7Pin),ch7Fall,FALLING);

  // Start timer
  ch7Timer = micros();
  
}

void ch7Fall() {

  // Attach interrupt to respective rise function
  attachInterrupt(digitalPinToInterrupt(ch7Pin),ch7Rise,RISING);
  
  // Save reading to local variable
  int reading = micros() - ch7Timer;

  // Only commit reading to global variable if it's within expected range
  if (reading >= ch7PulseMin - pulseTolerance && reading <= ch7PulseMax + pulseTolerance) { 
    ch7PulseRaw = reading;
  }
  
}

void ch8Rise() {
  
  // Attach interrupt to respective fall function
  attachInterrupt(digitalPinToInterrupt(ch8Pin),ch8Fall,FALLING);

  // Start timer
  ch8Timer = micros();
  
}

void ch8Fall() {

  // Attach interrupt to respective rise function
  attachInterrupt(digitalPinToInterrupt(ch8Pin),ch8Rise,RISING);
  
  // Save reading to local variable
  int reading = micros() - ch8Timer;

  // Only commit reading to global variable if it's within expected range
  if (reading >= ch8PulseMin - pulseTolerance && reading <= ch8PulseMax + pulseTolerance) { 
    ch8PulseRaw = reading;
  }
  
}
