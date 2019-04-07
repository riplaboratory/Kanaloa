int voltagePin = A0; 
float voltageBytes; 
float voltage;

void setup() {
  pinMode(voltagePin, INPUT);
  Serial.begin(9600);
}

void loop() {
  voltageBytes = analogRead(voltagePin);
  voltage = 0.0427*voltageBytes + 0.117; 
  Serial.print("Voltage: ");
  Serial.println(voltage);
  delay(100); 
}
