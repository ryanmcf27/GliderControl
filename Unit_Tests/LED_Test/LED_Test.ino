static const int LEDPin = 6;

void setup() {
  Serial.begin(115200);
  pinMode(LEDPin, OUTPUT);
}

void loop() {
  digitalWrite(LEDPin, LOW);
  Serial.println("LED Off :/");
  delay(500);
  digitalWrite(LEDPin, HIGH);
  Serial.println("LED On  :)");
  dellay(500);
}
