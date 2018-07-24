const int ch1Pin = 23;
const int ch2Pin = 22;
const int ch3Pin = 24; 
const int ch4Pin = 26;
const int ch5Pin = 28; 
const int ch6Pin = 30; 
const int ch7Pin = 32; 
const int ch8Pin = 34;
const int ch9Pin = 36;
const int ch10Pin = 38;
const int ch11Pin = 40; 
const int ch12Pin = 42;
const int ch13Pin = 44;
const int ch14Pin = 46;
const int ch15Pin = 48;
const int ch16Pin = 50; 
const int LED = 13; 


void setup() {
 pinMode(INPUT, ch1Pin);
 pinMode(INPUT, ch2Pin);
 pinMode(INPUT, ch3Pin);
 pinMode(INPUT, ch4Pin);
 pinMode(INPUT, ch5Pin);
 pinMode(INPUT, ch6Pin);
 pinMode(INPUT, ch7Pin);
 pinMode(INPUT, ch8Pin);
 pinMode(INPUT, ch9Pin);
 pinMode(INPUT, ch10Pin);
 pinMode(INPUT, ch11Pin);
 pinMode(INPUT, ch12Pin);
 pinMode(INPUT, ch13Pin);
 pinMode(INPUT, ch14Pin);
 pinMode(INPUT, ch15Pin);
 pinMode(INPUT, ch16Pin);
 pinMode(OUTPUT, LED);
 Serial.begin(9600);
}

void loop() {
  
  int ch1 = pulseIn(ch1Pin, HIGH, 50000);
  int ch2 = pulseIn(ch2Pin, HIGH, 50000);
  int ch3 = pulseIn(ch3Pin, HIGH, 50000);
  int ch4 = pulseIn(ch4Pin, HIGH, 50000);
  int ch5 = pulseIn(ch5Pin, HIGH, 50000);
  int ch6 = pulseIn(ch6Pin, HIGH, 50000);
  int ch7 = pulseIn(ch7Pin, HIGH, 50000);
  int ch8 = pulseIn(ch8Pin, HIGH, 50000);
  int ch9 = pulseIn(ch9Pin, HIGH, 50000);
  int ch10 = pulseIn(ch10Pin, HIGH, 50000);
  int ch11 = pulseIn(ch11Pin, HIGH, 50000);
  int ch12 = pulseIn(ch12Pin, HIGH, 50000);
  int ch13 = pulseIn(ch13Pin, HIGH, 50000);
  int ch14 = pulseIn(ch14Pin, HIGH, 50000);
  int ch15 = pulseIn(ch15Pin, HIGH, 50000);
  int ch16 = pulseIn(ch16Pin, HIGH, 50000);

  int myArray[16] = {ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12, ch13, ch14, ch15, ch16};

  for(int i=0; i<16; i++){
    if(i == 15){
      Serial.print("Ch");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.println(myArray[i]);      
    }
    else{
      Serial.print("Ch");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.print(myArray[i]);
      Serial.print(" --- ");
    }
  }
}
