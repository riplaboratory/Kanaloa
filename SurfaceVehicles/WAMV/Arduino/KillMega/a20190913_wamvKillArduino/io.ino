void readRemoteKill() {

  // Create local, non-volatile copy of pulse widths (QuickMedian does not like volatile arrays)
  int ch4PulseFilt[nMedian] = {};
  memcpy(ch4PulseFilt, ch4PulseArray, nMedian * 2);

  // Take median of pulse width arrays
  ch4PulseMedian = QuickMedian<int>::GetMedian(ch4PulseFilt, nMedian);

  // Determine mode from handheld receiver channel 4
  int switchUp = ch4PulseMax;
  int switchMid = round((ch4PulseMax+ch4PulseMin)/2);
  int switchDown = ch4PulseMin;
  if (ch4PulseMedian + pulseTolerance > switchUp && ch4PulseMedian - pulseTolerance < switchUp) {
    // Channel 4 switch is in the up position (unkill)
    remoteKillStatus = 0;
  }
  else if (ch4PulseMedian + pulseTolerance > switchDown && ch4PulseMedian - pulseTolerance < switchDown) {
    // Channel 4 switch is in the down position (kill)
    remoteKillStatus = 1;
  }
  else {
    // Invalid input from channel 4
    Serial.println("Remote kill switch (channel 4) reporting invalid input. Check transmitter connection...");
    remoteKillStatus = 1;
  }
  
}

void readPhysicalKill() {

  // Read state of physical kill switches
  if (digitalRead(physicalKillPin) == HIGH) {

    // Physical kill switch is unpressed (unkill)
    physicalKillStatus = 0;
    
  }
  else {

    // Physical kill switch is pressed (kill)
    physicalKillStatus = 1;
    
  }

}

void setKillStatus() {

  if (remoteKillStatus == 0 && physicalKillStatus == 0) {

    // Set kill status to 0
    killStatus = 0;

    // Unkill thrusters
    digitalWrite(q1KillPin,LOW);    // pull LOW to unkill
    digitalWrite(q2KillPin,LOW);    // pull LOW to unkill
    digitalWrite(q3KillPin,LOW);    // pull LOW to unkill
    digitalWrite(q4KillPin,LOW);    // pull LOW to unkill

    // Send unkill signal to main arduino
    digitalWrite(killCommPin,LOW);  // pull LOW to communicate unkill (as receiving pin on main arduino is set to INPUT_PULLUP)

    // Print debug
    Serial.println("System is UNKILLED");

  }

  else {

    // Set kill status to 1
    killStatus = 1;

    // Kill thrusters
    digitalWrite(q1KillPin,HIGH);     // set HIGH to kill
    digitalWrite(q2KillPin,HIGH);     // set HIGH to kill
    digitalWrite(q3KillPin,HIGH);     // set HIGH to kill
    digitalWrite(q4KillPin,HIGH);     // set HIGH to kill

    // Send unkill signal to main arduino
    digitalWrite(killCommPin,HIGH);   // set HIGH to communicate kill (as receiving pin on main arduino is set to INPUT_PULLUP)

    // Print debug
    Serial.println("System is KILLED");
    
  }

}
