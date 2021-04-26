void readKillArduino() {
  /*
     Read in information from the kill Arduino, and assign mode of operation accordingly.  Note that the mainMega cannot actually kill the system, since the actual kill is handled by the killMega.
  */

  if (digitalRead(killCommPin) == LOW) {
    killStatus = 0;       // killMega sending LOW (UNKILL)
  }
  else {
    killStatus = 1;       // killMega sending HIGH (KILL)
  }

}

void readHandheldReceiver() {
  /*
     Read in information from receiver using pin change interrupts.  Takes the median of multiple readings (determined by global variable nMedian) to help reject noise.
  */

  // Create local, non-volatile copy of pulse widths (QuickMedian does not like volatile arrays)
  int ch1PulseFilt[nMedian] = {};
  memcpy(ch1PulseFilt, ch1PulseArray, nMedian * 2);
  int ch2PulseFilt[nMedian] = {};
  memcpy(ch2PulseFilt, ch2PulseArray, nMedian * 2);
  int ch3PulseFilt[nMedian] = {};
  memcpy(ch3PulseFilt, ch3PulseArray, nMedian * 2);
  int ch5PulseFilt[nMedian] = {};
  memcpy(ch5PulseFilt, ch5PulseArray, nMedian * 2);
  int ch6PulseFilt[nMedian] = {};
  memcpy(ch6PulseFilt, ch6PulseArray, nMedian * 2);

  // Take median of pulse width arrays
  ch1PulseMedian = QuickMedian<int>::GetMedian(ch1PulseFilt, nMedian);
  ch2PulseMedian = QuickMedian<int>::GetMedian(ch2PulseFilt, nMedian);
  ch3PulseMedian = QuickMedian<int>::GetMedian(ch3PulseFilt, nMedian);
  ch5PulseMedian = QuickMedian<int>::GetMedian(ch5PulseFilt, nMedian);
  ch6PulseMedian = QuickMedian<int>::GetMedian(ch6PulseFilt, nMedian);
  
  //  // Print debug statement
    Serial.print("Ch1 (");
    Serial.print(ch1PulseMedian);
    Serial.print("); Ch2 (");
    Serial.print(ch2PulseMedian);
    Serial.print("); Ch3 (");
    Serial.print(ch3PulseMedian);
    Serial.print("); Ch5 (");
    Serial.print(ch5PulseMedian);
    Serial.print("); Ch6 (");
    Serial.print(ch6PulseMedian);
    Serial.println(");");

}

void readLowCurrentBatteryVoltage() {
  /*
     Reads low current battery voltage from voltage divider, and calculated true voltage
  */

  // Save low current battery read pin raw input (in bits)
  float voltLowCurrentBattBit = analogRead(voltLowCurrentPin);

  // Convert bits to voltage               
  float lowCurrentVoltDivider = voltLowCurrentBattBit * 5.0 / 1024;  // voltage at the voltage divider
  voltLowCurrentBatt = lowCurrentVoltDivider * 4.2;                  // low current battery voltage
  
  // Print debug statement
  // Serial.println(lowCurrentVoltDivider);
  // Serial.println(voltLowCurrentBatt);

}

void readMainBatteryVoltage() {
  /*
     Reads main battery voltage sent over from I2C communication
  */

  String voltageMsg = "";

  // Get voltage measurement as a String
  for (int j = 1; j < 5; j++) {
    voltageMsg += incoming[j];
  }

  // Convert voltage message to float
  previousVoltage = voltageMsg.toFloat();

  // Update voltage only when voltage measured changes
  if (voltMainBatt != previousVoltage && previousVoltage > 11.0) {
    voltMainBatt = previousVoltage;
  }

  // Uncomment Serial.print below to see if mainBatteryVoltage is being received correctly
  //Serial.print("Main Batt Voltage: "); Serial.println(voltMainBatt);
}
