#include <PWMServo.h> //must use this library because basic Servo.h Arduino library cannot run on Teensy 4.1's processor
static const int ServoPin = 2;
PWMServo rudderServo;                 //declare servo to control rudder

void setup() {
  pinMode(ServoPin, OUTPUT);
  //setup servo
  rudderServo.attach(ServoPin); //attach pin to servo object
}

void loop() {
  for(int i = 0; i <= 180; i++){
    Serial.print("Moving rudder to ");
    Serial.print(i);
    Serial.println(" degrees.");
    rudderServo.write(i);
    delay(50);
  }
  for(int i = 180; i >= 0; i--){
    Serial.print("Moving rudder to ");
    Serial.print(i);
    Serial.println(" degrees.");
    rudderServo.write(i);
    delay(50);
  }
}
