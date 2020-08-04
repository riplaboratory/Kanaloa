void setupGNSS(SFE_UBLOX_GPS &gnss){
  //----------------------------
  // Initialize local variables
  //----------------------------
  boolean response;            // variable used for gnss setup
  int minSurveyTime = 10;     // minimum amount of time [sec] for Survey-In to do
  float minSurveyAcc = 100.00;   // minimum positional accuracy [m] Survey-In must achieve
  //-------------------------------------------------------------
  // Connect to the Ublox module using Wire port at default value
  // and check to see if gnss is connected. 
  //-------------------------------------------------------------  
  gnss.begin(Wire);                                            // Setup GNSS for Serial communication at default value
  if(gnss.isConnected() == false){
    Serial.print(F("Initial GNSS setup failed. Freezing.\n")); // Print error message to oled
    
    while(1);                                                  // Freeze program (infinite loop)
  }
  else if(gnss.isConnected() == true){
    Serial.print(F("Setting up GNSS module...\n")); // Print message
  }

  //-----------------------------------------------------------
  //Check to see if Survey is in progress before initiating one
  //-----------------------------------------------------------
  response = gnss.getSurveyStatus(2000); // Query rtk module for SVIN status for time [ms] 
  
  if(response == false){
    Serial.print(F("Failed to get Survey In status. Freezing.\n"));
    while (1); // Freeze program (infinite loop)
  }
  //if(gnss.svin.active == true){
  //  Serial.print(F("Survey is already in progress. The GNSS will continue to Survey until the minimum time and minimum accuracy parameters are satisfied.\n"));
    //Serial.print(F("Minimum time[s]: "));
    //Serial.print(minSurveyTime);
    //Serial.print(F(".\tMinimum Accuracy[m]: "));
    //Serial.println(minSurveyAcc);
  //}
  else{
    response = gnss.enableSurveyMode(minSurveyTime, minSurveyAcc); // Enable Survey In. Minimum time is 300 seconds and minimum accuracy of 1.000 meters accuracy achieved.  
    if(response == false){
      Serial.print(F("Survey failed to start. Freezing.\n"));
      while(1);
    }
    Serial.print(F("Survey has begun and will continue to survey until minimum time and minimum accuracy parameters are satisfied.\n"));
    Serial.print(F("Minimum time[s]: "));
    Serial.print(minSurveyTime);
    Serial.print(F(".\tMinimum Accuracy[m]: "));
    Serial.println(minSurveyAcc);
  }
  gnss.setI2COutput(COM_TYPE_UBX);
  gnss.saveConfiguration();
}

void waitForSurvey(){
  boolean response;
  while(baseStation.svin.valid == false){
    response = baseStation.getSurveyStatus(2000); // query module for SVIN status with 2000 ms timeout.
    if(response == true){
      Serial.print(F("Time[s]: "));
      Serial.print((String)baseStation.svin.observationTime);
      Serial.print(F("\tAccuracy[m]: "));
      Serial.println((String)baseStation.svin.meanAccuracy);
    }
  }
  Serial.println(F("Survey finished. Enabling RTCM messages."));
}

void enableRTCM(){
  boolean response; 
  response = true;
  response &= baseStation.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1); //Enable message 1005 to output through I2C port, message every second
  response &= baseStation.enableRTCMmessage(UBX_RTCM_1077, COM_PORT_I2C, 1);
  response &= baseStation.enableRTCMmessage(UBX_RTCM_1087, COM_PORT_I2C, 1);
  response &= baseStation.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 10); //Enable message every 10 seconds
  
  if (response == true){
    Serial.print(F("\nRTCM messages enabled.\n"));
  }
  else{
    Serial.print(F("\nRTCM messages failed to enable. Freezing."));
    while (1); // freeze
  }
  Serial.print(F("Base survey complete! RTCM now broadcasting.\n"));
  baseStation.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3);
}
