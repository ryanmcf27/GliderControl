static const int LimitPin = 8;

void setup() {
  Serial.begin(115200);
  pinMode(LimitPin, INPUT_PULLUP);
}

void loop() {
  if(digitalRead(LimitPin) == HIGH) {
    Serial.println("Switch released! (HIGH)");
  } else {
    Serial.println("Switch being pressed... (LOW)");
  }
}
