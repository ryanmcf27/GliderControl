#include <PWMServo.h> //must use this library because basic Servo.h Arduino library cannot run on Teensy 4.1's processor
static const int ServoPin = 3;
PWMServo rudderServo;                 //declare servo to control rudder

void setup() {
  pinMode(ServoPin, OUTPUT);
  rudderServo.attach(ServoPin); //attach pin to servo object
  rudderServo.write(90);
  delay(15);
}

void loop() {
  for(int i = 3; i <= 15; i++){
    Serial.print("Moving rudder to ");
    Serial.print(i*10);
    Serial.println(" degrees.");
    rudderServo.write(i*10);
    // if(digitalRead(6)){
    //   digitalWrite(6, LOW);
    // } else {
    //   digitalWrite(6, HIGH);
    // }
    delay(500);
  }
  for(int i = 15; i >= 3; i--){
    Serial.print("Moving rudder to ");
    Serial.print(i*10);
    Serial.println(" degrees.");
    rudderServo.write(i*10);
    // if(digitalRead(6)){
    //   digitalWrite(6, LOW);
    // } else {
    //   digitalWrite(6, HIGH);
    // }
    delay(500);
  }
}
