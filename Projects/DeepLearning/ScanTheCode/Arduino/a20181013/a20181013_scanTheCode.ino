// Pins
const int red = 4;
const int blue = 5;
const int green = 6; 

void setup(){
  pinMode(red, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(green, OUTPUT);
}

void loop(){
  delay(1000);
  digitalWrite(red, HIGH);
  digitalWrite(blue, LOW);
  digitalWrite(green, LOW);
  delay(1000);
  digitalWrite(red, LOW);
  digitalWrite(blue, HIGH);
  digitalWrite(green, LOW);
  delay(1000);
  digitalWrite(red, LOW);
  digitalWrite(blue, LOW);
  digitalWrite(green, HIGH);
  delay(1000);
  digitalWrite(red, LOW);
  digitalWrite(blue, LOW);
  digitalWrite(green, LOW);
  delay(1000);
}
