/* Interrupt Service Routines are functions that execute in response to a system interrupt. 
 * These functions have some unique limitations most other functions to not have.  
 * 
 * Note that the Arduino Mega 2560 can use pin change interrupts on pins 10,11,12,13,50,51,52,53,A8(62),A9(63),A10(64),A11(65),A12(66),A13(67),A14(68),A15(69). 
*/

void ch4Change() {

  // Check the current state of the pin
  ch4State = digitalRead(ch4Pin);

  if (ch4State == 1) {
    // Pin is high, this was probably a rise event

    // Save current time to timer
    ch4Timer = micros();

  }

  else {
    // Pin is low, this was probably a fall event

    // Subtract ch1Time from current time
    ch4PulseRaw = micros() - ch4Timer;

    // Only commit reading to memory if it's within expected range
    if (ch4PulseRaw >= ch4PulseMin - pulseTolerance && ch4PulseRaw <= ch4PulseMax + pulseTolerance) {

      // Move new measurement into circular buffer array
      memcpy(ch4PulseArray, &ch4PulseArray[1], sizeof(ch4PulseArray) - sizeof(int));
      ch4PulseArray[nMedian - 1] = ch4PulseRaw;

    }

  }

}
