void processGotForward(unsigned int value){
  serialSpeedCommand = value;
  Serial.print("Serial Speed Command: ");
  Serial.println(serialSpeedCommand);
}

void processGotReverse(unsigned int value){
  serialSpeedCommand = -1*value;
  Serial.print("Serial Speed Command: ");
  Serial.println(serialSpeedCommand);
}

void handlePreviousState(){
  switch(state){
    case GOT_FORWARD:
      processGotForward(currentSerialValue);
      break;
    case GOT_REVERSE:
      processGotReverse(currentSerialValue);
      break;
  }
}

void processIncomingByte(byte c){
  if(isdigit(c)){
    currentSerialValue *= 10;
    currentSerialValue += c & 0xf;
  }
  else{
    handlePreviousState();
    switch(c){
      case 'f':
        state = GOT_FORWARD;
        currentSerialValue = 0;
        break;
      case 'r':
        state = GOT_REVERSE;
        currentSerialValue = 0;
        break;
      default:
        state = NONE;
        currentSerialValue = 0;
        break;
    }
  }
}

