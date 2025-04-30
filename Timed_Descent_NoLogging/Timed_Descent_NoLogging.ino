/*
  Timed_Descent_NoLogging

  Simplified code (only timing, no logging!!) for Iowa State University's 2024-2025 Design Build Fly (DBF) team's X-1 glider

  This program only uses the limit switch, servo, and LEDs (does not use GPS, IMU, or SD)

  Backup in case parts break or we have technical issues with data logging
  
*/

/* Libraries */
#include <TaskScheduler.h> //for more info refer to source repository: https://github.com/arkhipenko/TaskScheduler/tree/master
#include <PWMServo.h> //must use this library because basic Servo.h Arduino library cannot run on Teensy 4.1's processor

/* Method Prototypes */
void moveRudder(int angle);


// global flag to tell whether (true) or not (false) the limit switch has been triggered 
bool limitTriggeredFlag = true;
unsigned long limitTriggeredTime = 0;

// set up enum for the glider's flight state, initialize to PRERELEASE state
enum {PRERELEASE, DESCENT_OUT, DESCENT_TURNAROUND, DESCENT_RETURN, DESCENT_CIRCLE, ERROR_STATE} flightState = PRERELEASE;

/* Constants and Pins */
static const int ServoPin = 10, LEDPin = 6, LimitPin = 8;
static const uint32_t MONITOR_BAUD = 115200;
static const int LIMIT_TRIGGERED_BUFFER_MILLIS = 1000;

//servo info
#define SERVO_CENTER_DEGREES 90
#define SERVO_TURN_RIGHT_SOFT_DEGREES 150
#define SERVO_TURN_RIGHT_HARD_DEGREES 180
#define SERVO_TURN_LEFT_SOFT_DEGREES 30
#define SERVO_TURN_LEFT_HARD_DEGREES 0
#define RUDDER_RANGE_DEGREES 180  //Limits the servo's movement from (90 - RUDDER_RANGE_DEGREES) to (90 + RUDDER_RANGE_DEGREES), corrects if invalid input is given to moveRudder function

//static const int RUDDER_RANGE_DEGREES = 90; //Limits the servo's movement from (90 - RUDDER_RANGE_DEGREES) to (90 + RUDDER_RANGE_DEGREES), corrects if invalid input is given to moveRudder function

//timing info
#define STRAIGHT_POST_RELEASE_MILLIS 2000
#define TURNAROUND_MILLIS 5000
#define STRAIGHT_POST_TURNAROUND_MILLIS 2000
//will be used as "stopwatches" to achieve the timings outlined above for each stage of the flight
int flightStateTimingStart, flightStateTimingEnd;
#define LED_DELAY_MILLIS 200

/* Declaring Components */
PWMServo rudderServo;                 //declare servo to control rudder

void setup() {
  //setup Serial communication
  Serial.begin(MONITOR_BAUD);

  //setup pins
  pinMode(LEDPin, OUTPUT);
  pinMode(LimitPin, INPUT_PULLUP);
  pinMode(ServoPin, OUTPUT);

  //setup servo
  rudderServo.attach(ServoPin); //attach pin to servo object
  moveRudder(SERVO_CENTER_DEGREES);               //set servo to center position

  delay(1000);
}

void loop() {
  switch(flightState){
      case PRERELEASE:
      Serial.println("prerelease");
      //checks to see if X-1 has been released based on limit switch
      if(digitalRead(LimitPin) == HIGH){
        //switch opened (X-1 likely released)
        if(!limitTriggeredFlag){
          //"starts the stopwatch" for how long the limit switch has been open
          limitTriggeredFlag = true;
          limitTriggeredTime = millis();
        } else {
          unsigned long currentMillis = millis();
          //if the limit switch has been opened for specified time
          if((currentMillis - limitTriggeredTime) >= LIMIT_TRIGGERED_BUFFER_MILLIS){
            //start strobing LEDs by enabling strobe task
            //strobeTask.enable();
            //update flightState
            flightState = DESCENT_OUT;
          }
        }
      } else {
        //switch closed
        limitTriggeredFlag = false;
      }
      break;
    case DESCENT_OUT:
      Serial.println("descent out");
      //fly straight until reaching turn line 
      moveRudder(SERVO_CENTER_DEGREES);
      flightStateTimingStart = millis();
      flightStateTimingEnd = flightStateTimingStart;
      //wait in this loop for time specified by STRAIGHT_POST_RELEASE_MILLIS
      while(flightStateTimingEnd - flightStateTimingStart < STRAIGHT_POST_RELEASE_MILLIS){
        flightStateTimingEnd = millis();
        
        //Serial.println("Strobe");
        // Stobe LEDs by setting them to HIGH if they were LOW, and LOW if they were HIGH
        if(digitalRead(LEDPin) == LOW){
          digitalWrite(LEDPin, HIGH);
          delay(LED_DELAY_MILLIS);
        } else {
          digitalWrite(LEDPin, LOW);
          delay(LED_DELAY_MILLIS);
        }
      }
      flightState = DESCENT_TURNAROUND;
      break;
    case DESCENT_TURNAROUND:
      Serial.println("descent turnaround");
      moveRudder(SERVO_TURN_LEFT_SOFT_DEGREES); //CAN CHANGE TO TURN RIGHT
      flightStateTimingStart = millis();
      flightStateTimingEnd = flightStateTimingStart;
      //wait in this loop for time specified by TURNAROUND_MILLIS
      while(flightStateTimingEnd - flightStateTimingStart < TURNAROUND_MILLIS){
        flightStateTimingEnd = millis();

        //Serial.println("Strobe");
        // Stobe LEDs by setting them to HIGH if they were LOW, and LOW if they were HIGH
        if(digitalRead(LEDPin) == LOW){
          digitalWrite(LEDPin, HIGH);
          delay(LED_DELAY_MILLIS);
        } else {
          digitalWrite(LEDPin, LOW);
          delay(LED_DELAY_MILLIS);
        }
      }
      flightState = DESCENT_RETURN;
      break;
    case DESCENT_RETURN:
      Serial.println("descent return");
      //fly straight until reaching finish line
      moveRudder(SERVO_CENTER_DEGREES);
      flightStateTimingStart = millis();
      flightStateTimingEnd = flightStateTimingStart;
      //wait in this loop for time specified by STRAIGHT_POST_TURNAROUND_MILLIS
      while(flightStateTimingEnd - flightStateTimingStart < STRAIGHT_POST_TURNAROUND_MILLIS){
        flightStateTimingEnd = millis();

        //Serial.println("Strobe");
        // Stobe LEDs by setting them to HIGH if they were LOW, and LOW if they were HIGH
        if(digitalRead(LEDPin) == LOW){
          digitalWrite(LEDPin, HIGH);
          delay(LED_DELAY_MILLIS);
        } else {
          digitalWrite(LEDPin, LOW);
          delay(LED_DELAY_MILLIS);
        }
      }
      flightState = DESCENT_CIRCLE;
      break;
    case DESCENT_CIRCLE:
      Serial.println("descent circle");
      //set rudder to circle in scoring zone until landing
      moveRudder(SERVO_TURN_LEFT_HARD_DEGREES); //CAN CHANGE TO TURN RIGHT
      while(1){
        //infinite loop, circle until landing
        //Serial.println("Strobe");
        // Stobe LEDs by setting them to HIGH if they were LOW, and LOW if they were HIGH
        if(digitalRead(LEDPin) == LOW){
          digitalWrite(LEDPin, HIGH);
          delay(LED_DELAY_MILLIS);
        } else {
          digitalWrite(LEDPin, LOW);
          delay(LED_DELAY_MILLIS);
        }
      }
      break;
    case ERROR_STATE:
      Serial.println("In ERROR flight state!!!");
      break;
  }
}

