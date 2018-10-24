/* Interrupt Service Routines are functions that execute in response to a system interrupt. 
 * These functions have some unique limitations most other functions to not have.  You can read more here:  
 *  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 *  
 *  If using external interrupts, the pins available on the MEGA 2560 are 2,3,18,19,20,21
 *  If using pin change interrupts, you must know the  the pins available on the MEGA 2560 are A8,A9,A10,A11,A12,A13,A14,A15
*/

void ch1Rise() {
  attachInterrupt(digitalPinToInterrupt(ch1Pin),ch1Fall,FALLING);
  ch1Timer = micros();
}

void ch1Fall() {
  attachInterrupt(digitalPinToInterrupt(ch1Pin),ch1Rise,RISING);
  ch1PulseWidth = micros() - ch1Timer;
}

void ch4Rise() {
  attachInterrupt(digitalPinToInterrupt(ch4Pin),ch4Fall,FALLING);
  ch4Timer = micros();
}

void ch4Fall() {
  attachInterrupt(digitalPinToInterrupt(ch4Pin),ch4Rise,RISING);
  ch4PulseWidth = micros() - ch4Timer;
}

void ch5Rise() {
  attachInterrupt(digitalPinToInterrupt(ch5Pin),ch5Fall,FALLING);
  ch5Timer = micros();
}

void ch5Fall() {
  attachInterrupt(digitalPinToInterrupt(ch5Pin),ch5Rise,RISING);
  ch5PulseWidth = micros() - ch5Timer;
}

void ch6Rise() {
  attachInterrupt(digitalPinToInterrupt(ch6Pin),ch6Fall,FALLING);
  ch6Timer = micros();
}

void ch6Fall() {
  attachInterrupt(digitalPinToInterrupt(ch6Pin),ch6Rise,RISING);
  ch6PulseWidth = micros() - ch6Timer;
}

void ch7Rise() {
  attachInterrupt(digitalPinToInterrupt(ch7Pin),ch7Fall,FALLING);
  ch7Timer = micros();
}

void ch7Fall() {
  attachInterrupt(digitalPinToInterrupt(ch7Pin),ch7Rise,RISING);
  ch7PulseWidth = micros() - ch7Timer;
}

void ch8Rise() {
  attachInterrupt(digitalPinToInterrupt(ch8Pin),ch8Fall,FALLING);
  ch8Timer = micros();
}

void ch8Fall() {
  attachInterrupt(digitalPinToInterrupt(ch8Pin),ch8Rise,RISING);
  int currentTime = micros();
  if ((currentTime - ch8Timer) > 400 && (currentTime - ch8Timer) < 1600) {
    ch8PulseWidth = micros() - ch8Timer;
  }
}
