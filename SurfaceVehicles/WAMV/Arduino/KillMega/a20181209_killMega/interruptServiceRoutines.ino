/* Interrupt Service Routines are functions that execute in response to a system interrupt. 
 * These functions have some unique limitations most other functions to not have.  You can read more here:  
 *  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 *  
 *  If using external interrupts, the pins available on the MEGA 2560 are 2,3,18,19,20,21
 *  If using pin change interrupts, you must know the  the pins available on the MEGA 2560 are A8,A9,A10,A11,A12,A13,A14,A15
*/

void ch8Rise() {
  attachInterrupt(digitalPinToInterrupt(ch8Pin),ch8Fall,FALLING);
  ch8Timer = micros();
}

void ch8Fall() {
  attachInterrupt(digitalPinToInterrupt(ch8Pin),ch8Rise,RISING);
  ch8PulseWidth = micros() - ch8Timer;
}
