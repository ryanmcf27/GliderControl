#include <PWMServo.h> //must use this library because basic Servo.h Arduino library cannot run on Teensy 4.1's processor
static const int ServoPin = 2;
PWMServo rudderServo;                 //declare servo to control rudder

void setup() {
  pinMode(ServoPin, OUTPUT);
  //setup servo
  rudderServo.attach(ServoPin); //attach pin to servo object
  rudderServo.write(90);
  delay(15);
}

void loop() {
  for(int i = 0; i <= 18; i++){
    Serial.print("Moving rudder to ");
    Serial.print(i*10);
    Serial.println(" degrees.");
    rudderServo.write(i*10);
    delay(1000);
  }
  for(int i = 18; i >= 0; i--){
    Serial.print("Moving rudder to ");
    Serial.print(i*10);
    Serial.println(" degrees.");
    rudderServo.write(i*10);
    delay(1000);
  }
}
