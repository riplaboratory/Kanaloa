void joy2Setpoint() {
  /*
     Take readings fromt receiver joystick channels, and convert to setpoint thruster values from -1000 to 1000.
  */

  // Grab ch1, ch4, and ch6 from filtering function
  int ch1Pulse = ch1PulseMedian;    // yaw (right stick left-right)
  int ch2Pulse = ch2PulseMedian;    // sway (left stick left-right)

  // Pulse width variables
  int ch1PulseNeutral = round((ch1PulseMax - ch1PulseMin) / 2) + ch1PulseMin;
  int ch2PulseNeutral = round((ch2PulseMax - ch2PulseMin) / 2) + ch2PulseMin;

  // Remove the deadzone
  if (ch1Pulse < ch1PulseNeutral + round(pulseDeadzone / 2) && ch1Pulse > ch1PulseNeutral - round(pulseDeadzone / 2)) {
    ch1Pulse = ch1PulseNeutral;
  }
  if (ch2Pulse < ch2PulseNeutral + round(pulseDeadzone / 2) && ch2Pulse > ch2PulseNeutral - round(pulseDeadzone / 2)) {
    ch2Pulse = ch2PulseNeutral;
  }

  // Map joystick inputs from -1000 to 1000
  int ch1Map = map(ch1Pulse, ch1PulseMin, ch1PulseMax, -1000, 1000); // surge (positive forward, negative backward)
  int ch2Map = map(ch2Pulse, ch2PulseMin, ch2PulseMax, 1000, -1000); // yaw (positive CCW, negative CW)

  // Calculate surge sway and yaw components
  int surgeLeft = ch1Map;     // surge (forward positive)
  int surgeRight = ch1Map;
  int yawLeft = -ch2Map;       // yaw (CCW positive)
  int yawRight = ch2Map;

  // Map thruster components from -1000 to 1000
  leftThrusterSetpoint = constrain(surgeLeft + yawLeft, -1000, 1000);
  rightThrusterSetpoint = constrain(surgeRight + yawRight, -1000, 1000);

}
