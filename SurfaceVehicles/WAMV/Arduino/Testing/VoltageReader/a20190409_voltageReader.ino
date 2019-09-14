int voltagePin = A1; 
float voltageBytes; 
float voltage;

void setup() {
  pinMode(voltagePin, INPUT);
  Serial.begin(9600);
}

void loop() {
  voltageBytes = analogRead(voltagePin);
  voltage = 0.0401*voltageBytes + 0.0591; 
  Serial.print("Voltage: ");
  Serial.println(voltage);
  delay(100); 
}
