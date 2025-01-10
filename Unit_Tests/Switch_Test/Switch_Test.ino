static const int LimitPin = 8;

void setup() {
  pinMode(LimitPin, INPUT);
}

void loop() {
  if(digitalRead(LimitPin) == HIGH) {
    Serial.println("Limit switch HIGH");
  } else {
    Serial.println("Limit switch LOW");
  }
}
