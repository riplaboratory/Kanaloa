void determineMode() {
  /*
     Determine operation mode (kill, battery check, and auto/manual) based on inputs
  */

  // Determine mode from handheld receiver channel 3
  int switchUp = ch3PulseMin;
  int switchMid = round((ch3PulseMax + ch3PulseMin) / 2);
  int switchDown = ch3PulseMax;
  if (ch3PulseMedian + pulseTolerance > switchUp && ch3PulseMedian - pulseTolerance < switchUp) {
    // Channel 3 switch is in the up position (autonomous)
    mode = 2;
  }
  else if (ch3PulseMedian + pulseTolerance > switchMid && ch3PulseMedian - pulseTolerance < switchMid) {
    // Channel 3 switch is in the middle position (manual)
    mode = 1;
  }
  else if (ch3PulseMedian + pulseTolerance > switchDown && ch3PulseMedian - pulseTolerance < switchDown) {
    // Channel 3 switch is in the down position (manual)
    mode = 1;
  }
  else {
    // Invalid input from channel 3
    Serial.println("Mode switch (channel 3) reporting invalid input; check transmitter connection...");
    mode = 0;
  }

  // Determine main battery check mode from receiver channel 5
  switchUp = ch5PulseMin;
  switchMid = round((ch5PulseMax + ch5PulseMin) / 2);
  switchDown = ch5PulseMax;
  if (ch5PulseMedian + pulseTolerance > switchUp && ch5PulseMedian - pulseTolerance < switchUp) {
    // Channel 5 switch is in the up position (yes to main battery check)
    mainBattCheck = 1;
  }
  else if (ch5PulseMedian + pulseTolerance > switchMid && ch5PulseMedian - pulseTolerance < switchMid) {
    // Channel 5 switch is in the middle position (no to main battery check)
    mainBattCheck = 2;
  }
  else if (ch5PulseMedian + pulseTolerance > switchDown && ch5PulseMedian - pulseTolerance < switchDown) {
    // Channel 5 switch is in the down position (no to main battery check)
    mainBattCheck = 2;
  }
  else {
    // Invalid input from channel 5
    Serial.println("Main battery check switch (channel 5) reporting invalid input; check transmitter connection...");
    mainBattCheck = 0;
  }

  // Determine main battery check mode from receiver channel 6
  switchUp = ch6PulseMin;
  switchMid = round((ch6PulseMax + ch6PulseMin) / 2);
  switchDown = ch6PulseMax;
  if (ch6PulseMedian + pulseTolerance > switchUp && ch6PulseMedian - pulseTolerance < switchUp) {
    // Channel 6 switch is in the up position (yes to low current battery check)
    lowCurrentBattCheck = 1;
  }
  else if (ch6PulseMedian + pulseTolerance > switchMid && ch6PulseMedian - pulseTolerance < switchMid) {
    // Channel 6 switch is in the middle position (no to low current battery check)
    lowCurrentBattCheck = 2;
  }
  else if (ch6PulseMedian + pulseTolerance > switchDown && ch6PulseMedian - pulseTolerance < switchDown) {
    // Channel 6 switch is in the down position (no to low current battery check)
    lowCurrentBattCheck = 2;
  }
  else {
    // Invalid input from channel 6
    Serial.println("Low current battery check switch (channel 6) reporting invalid input; check transmitter connection...");
    lowCurrentBattCheck = 0;
  }

}

void controlLight() {
  /*
     Changes light in response to mode of operation.
  */

  if (killStatus == 1) {
    // System is killed, solid red.
    changeLight(2);
  }
  else {
    // System is unkilled, proceed.
    if (mode == 1) {
      // Manual mode engaged, solid yellow.
      changeLight(5);
    }
    else if (mode == 2) {
      // Autonomous mode engaged, solid red.
      changeLight(4);
    }
    else {
      // Something else, error, solid white.
      Serial.println("Mode error!  Check trasnmitter connection!");
      changeLight(1);
    }
  }
  




    
    if (mainBattCheck == 1) {
      if (lowCurrentBattCheck == 1) {
        // Main battery check and low battery check mode simultaneously. Flash white to indicate battery check mode error.
        changeLight(1);
        delay(100);
        changeLight(0);
        delay(100);
      }
      else if (lowCurrentBattCheck == 2) {
        // Main battery check mode, flash light as appropriate to main battery level.
        if (voltMainBatt > 29.4) {
          // Flash green.
          changeLight(3);
          delay(100);
          changeLight(0);
          delay(100);
        }
        else if (voltMainBatt <= 28.42 && voltMainBatt > 27.44) {
          // Flash blue then green.
          changeLight(3);
          delay(100);
          changeLight(4);
          delay(100);
        }
        else if (voltMainBatt <= 27.44 && voltMainBatt > 26.46) {
          // Flash blue.
          changeLight(4);
          delay(100);
          changeLight(0);
          delay(100);
        }
        else if (voltMainBatt <= 26.46 && voltMainBatt > 25.48) {
          // Flash blue then red.
          changeLight(4);
          delay(100);
          changeLight(2);
          delay(100);
        }
        else {
          // Flash red.
          changeLight(2);
          delay(100);
          changeLight(0);
          delay(100);
        }
      }
      else {
        // Low current battery check mode error, flash white to indicate this.
        Serial.println("Low current battery check error!");
        changeLight(1);
      }
    }
    else if (mainBattCheck == 2) {
      if (lowCurrentBattCheck == 1) {
        // Low current battery check mode, flash light as appropriate to low current battery level.
        if (voltLowCurrentBatt > 16.24) {
          // Flash green.
          changeLight(3);
          delay(100);
          changeLight(0);
          delay(100);
        }
        else if (voltLowCurrentBatt <= 16.24 && voltLowCurrentBatt > 15.56) {
          // Flash blue then green.
          changeLight(3);
          delay(100);
          changeLight(4);
          delay(100);
        }
        else if (voltLowCurrentBatt <= 15.68 && voltLowCurrentBatt > 15.12) {
          // Flash blue.
          changeLight(4);
          delay(100);
          changeLight(0);
          delay(100);
        }
        else if (voltLowCurrentBatt <= 15.12 && voltLowCurrentBatt > 14.56) {
          // Flash blue then red.
          changeLight(4);
          delay(100);
          changeLight(2);
          delay(100);
        }
        else {
          // Flash red.
          changeLight(2);
          delay(100);
          changeLight(0);
          delay(100);
        }
      }
      else if (lowCurrentBattCheck == 2) {
        // Neither of the battery check modes are engaged, proceed to normal operation.
        if (mode == 1) {
          // Manual mode engaged, solid yellow.
          changeLight(5);
        }
        else if (mode == 2) {
          // Autonomous mode engaged, solid red.
          changeLight(4);
        }
        else {
          // Something else, error, solid white.
          Serial.println("Mode error!  Check trasnmitter connection!");
          changeLight(1);
        }
      }
      else {
        // Low current battery check mode error, flash white to indicate this.
        Serial.println("Low current battery check error!");
      }
    }
    else {
      // Main battery check mode error, flash white to indicate this.
      Serial.println("Main battery check error!");
    }
  }

//  if (killStatus == 1) {
//    // System is killed, solid red.
//    changeLight(2);
//  }
//  else {  
//    if (mainBattCheck == 1) {
//      if (lowCurrentBattCheck == 1) {
//        // Main battery check and low battery check mode simultaneously. Flash white to indicate battery check mode error.
//        changeLight(1);
//        delay(100);
//        changeLight(0);
//        delay(100);
//      }
//      else if (lowCurrentBattCheck == 2) {
//        // Main battery check mode, flash light as appropriate to main battery level.
//        if (voltMainBatt > 29.4) {
//          // Flash green.
//          changeLight(3);
//          delay(100);
//          changeLight(0);
//          delay(100);
//        }
//        else if (voltMainBatt <= 28.42 && voltMainBatt > 27.44) {
//          // Flash blue then green.
//          changeLight(3);
//          delay(100);
//          changeLight(4);
//          delay(100);
//        }
//        else if (voltMainBatt <= 27.44 && voltMainBatt > 26.46) {
//          // Flash blue.
//          changeLight(4);
//          delay(100);
//          changeLight(0);
//          delay(100);
//        }
//        else if (voltMainBatt <= 26.46 && voltMainBatt > 25.48) {
//          // Flash blue then red.
//          changeLight(4);
//          delay(100);
//          changeLight(2);
//          delay(100);
//        }
//        else {
//          // Flash red.
//          changeLight(2);
//          delay(100);
//          changeLight(0);
//          delay(100);
//        }
//      }
//      else {
//        // Low current battery check mode error, flash white to indicate this.
//        Serial.println("Low current battery check error!");
//        changeLight(1);
//      }
//    }
//    else if (mainBattCheck == 2) {
//      if (lowCurrentBattCheck == 1) {
//        // Low current battery check mode, flash light as appropriate to low current battery level.
//        if (voltLowCurrentBatt > 16.24) {
//          // Flash green.
//          changeLight(3);
//          delay(100);
//          changeLight(0);
//          delay(100);
//        }
//        else if (voltLowCurrentBatt <= 16.24 && voltLowCurrentBatt > 15.56) {
//          // Flash blue then green.
//          changeLight(3);
//          delay(100);
//          changeLight(4);
//          delay(100);
//        }
//        else if (voltLowCurrentBatt <= 15.68 && voltLowCurrentBatt > 15.12) {
//          // Flash blue.
//          changeLight(4);
//          delay(100);
//          changeLight(0);
//          delay(100);
//        }
//        else if (voltLowCurrentBatt <= 15.12 && voltLowCurrentBatt > 14.56) {
//          // Flash blue then red.
//          changeLight(4);
//          delay(100);
//          changeLight(2);
//          delay(100);
//        }
//        else {
//          // Flash red.
//          changeLight(2);
//          delay(100);
//          changeLight(0);
//          delay(100);
//        }
//      }
//      else if (lowCurrentBattCheck == 2) {
//        // Neither of the battery check modes are engaged, proceed to normal operation.
//        if (mode == 1) {
//          // Manual mode engaged, solid yellow.
//          changeLight(5);
//        }
//        else if (mode == 2) {
//          // Autonomous mode engaged, solid red.
//          changeLight(4);
//        }
//        else {
//          // Something else, error, solid white.
//          Serial.println("Mode error!  Check trasnmitter connection!");
//          changeLight(1);
//        }
//      }
//      else {
//        // Low current battery check mode error, flash white to indicate this.
//        Serial.println("Low current battery check error!");
//      }
//    }
//    else {
//      // Main battery check mode error, flash white to indicate this.
//      Serial.println("Main battery check error!");
//    }
//  }
}

void changeLight(int lightColor) {
  /*
     Changes the light color based on input "lightColor")
  */

  // light color variable (0 = off, 1 = white, 2 = red, 3 = green, 4 = blue, 5 = yellow, 6 = light blue)

  if (lightColor == 0) {
    digitalWrite(redPin, HIGH);
    digitalWrite(bluePin, HIGH);
    digitalWrite(greenPin, HIGH);
  }
  else if (lightColor == 1) {
    digitalWrite(redPin, LOW);
    digitalWrite(bluePin, LOW);
    digitalWrite(greenPin, LOW);
  }
  else if (lightColor == 2) {
    digitalWrite(redPin, LOW);
    digitalWrite(bluePin, HIGH);
    digitalWrite(greenPin, HIGH);
  }
  else if (lightColor == 3) {
    digitalWrite(redPin, HIGH);
    digitalWrite(bluePin, HIGH);
    digitalWrite(greenPin, LOW);
  }
  else if (lightColor == 4) {
    digitalWrite(redPin, HIGH);
    digitalWrite(bluePin, LOW);
    digitalWrite(greenPin, HIGH);
  }
  else if (lightColor == 5) {
    digitalWrite(redPin, LOW);
    digitalWrite(bluePin, HIGH);
    digitalWrite(greenPin, LOW);
  }
  else if (lightColor == 6) {
    digitalWrite(redPin, HIGH);
    digitalWrite(bluePin, LOW);
    digitalWrite(greenPin, LOW);
  }
}

void createI2cMsg() {
  /*
     Create message to send over I2C (based on the thruster setpoints)
  */

  // Create string message to send over I2C
  q1Msg += commandToMsg(leftThrusterSetpoint);   // Q1 will act as rightServoOut
  q2Msg += commandToMsg(rightThrusterSetpoint);  // Q2 will act as leftServoOut
  //  q3Msg += commandToMsg(q3Out);
  //  q4Msg += commandToMsg(q4Out);

  //  // Print debug statements
  //  Serial.print("q1Msg: ");
  //  Serial.println(q1Msg);
  //  Serial.print("q2Msg: ");
  //  Serial.println(q2Msg);

  // Ensure string message being set is correct size
  tempMsg += q1Msg;
  tempMsg += q2Msg;
  //  temp += q3Msg;
  //  temp += q4Msg;
  if (sizeof(tempMsg) == 6) {
    motorCmds = tempMsg;
  }

  // Reset global serial messages
  dir = 'N';
  q1Msg = "Q1";
  q2Msg = "Q2";
  q3Msg = "Q3";
  q4Msg = "Q4";
  tempMsg = "";

}

// Create string to send over I2C based on thruster setpoints
String commandToMsg(int motor) {
  /*
     Create string to send over I2C based on thruster setpoints.
     Checks to see if the motor command is greater/less than neutral point to determine whether F/R (forward/reverse) direction character should be used.

     Arguments
       int motor: motor command

     Output:
       String msg: motor magnitude and direction as a String
  */

  // Create output variable
  String msg;

  // Logic for building string
  if (abs(motor) < 1000 && abs(motor) >= 100) {
    msg = "0" + String(abs(motor));
  }
  else if (abs(motor) < 100 && abs(motor) >= 10) {
    msg = "00" + String(abs(motor));
  }
  else if (abs(motor) < 10) {
    msg = "000" + String(abs(motor));
  }
  else {
    msg = String(abs(motor));
  }
  if (motor < 0 && motor >= -1000) {
    dir = 'R';
    return msg += dir;
  }
  if (motor > 0 && motor <= 1000) {
    dir = 'F';
    return msg += dir;
  }
  else {
    dir = 'N';
    return msg += dir;
  }

}
