/*
  Simply_Timed_Descent

  Simplified code for Iowa State University's 2024-2025 Design Build Fly (DBF) team's X-1 glider
  
*/

/* Libraries */
#include <TaskScheduler.h> //for more info refer to source repository: https://github.com/arkhipenko/TaskScheduler/tree/master
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "src/MPU6050/MPU6050.h"
#include <PWMServo.h> //must use this library because basic Servo.h Arduino library cannot run on Teensy 4.1's processor
#include <SD.h>
#include <SPI.h>

/* Method Prototypes */
void updateGPS();
void strobe();
void moveRudder(int angle);
void updateIMU();
void IMUinit();
void calculate_IMU_error();
void outputToSD();

// set up a global struct to store our sensor data
struct SensorDataStruct {
  float gpsLat = 0.0;     //X-1's latitude (degrees)
  float gpsLng = 0.0;     //X-1's longitude (degrees)
  float roll = 0.0;       //roll angle (degrees) - may need to add offset if IMU isn't mounted flat
  float pitch = 0.0;      //pitch angle (degrees) - may need to add offset if IMU isn't mounted flat
  float yaw = 0.0;        //yaw angle (degrees) - may need to add offset if IMU isn't mounted flat
  float accelX = 0.0;     //X-1's acceleration in the X direction
  float accelY = 0.0;     //X-1's acceleration in the Y direction
  float accelZ = 0.0;     //X-1's acceleration in the Z direction
};

// global variables to store the coordinates of our target location
//float targetLat = 32.265617;
//float targetLng = -111.273524;

// global flag to tell whether (true) or not (false) the limit switch has been triggered 
bool limitTriggeredFlag = true;
unsigned long limitTriggeredTime = 0;

// global variable to store the yaw (direction our glider is facing) when released
float releasedYaw = 0.0;

// set up enum for the glider's flight state, initialize to PRERELEASE state
enum {PRERELEASE, DESCENT_OUT, DESCENT_TURNAROUND, DESCENT_RETURN, DESCENT_CIRCLE, ERROR_STATE} flightState = PRERELEASE;

/* Constants and Pins */
static const int RXPin = 0, TXPin = 1, ServoPin = 3, LEDPin = 5, LimitPin = 8;
static const uint32_t MONITOR_BAUD = 115200, GPS_BAUD = 9600;
static const int RUDDER_RANGE_DEGREES = 180; //Limits the servo's movement from (90 - RUDDER_RANGE_DEGREES) to (90 + RUDDER_RANGE_DEGREES), corrects if invalid input is given to moveRudder function
static const int LIMIT_TRIGGERED_BUFFER_MILLIS = 1000;
//conversion factor to help us find the equation of the lines that we'd like the X-1 to turn around at and to start circular descent 
static const float DEGREES_PER_FOOT_FROM_START_LINE = 1.109787E-5;     //NOTE this isn't how longitude and latitude work precisely, but will be fine on this small bit of the global scale
static const int TURNAROUND_FEET_FROM_START_LINE = 200;
//values for storing data to the SD card
const int CS = BUILTIN_SDCARD;
const char* FILENAME = "Teensy_Descent_OUT.txt";


//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these error values, then comment out calculate_IMU_error()
float AccErrorX = 0.06;
float AccErrorY = -0.29;
float AccErrorZ = -0.06;
float GyroErrorX = -2.68;
float GyroErrorY = 1.51;
float GyroErrorZ = 1.92;

static const float ACCEL_SCALE_FACTOR = 16384.0;    //full scale accelerometer range (16384.0 for +-2Gs)
static const float GYRO_SCALE_FACTOR = 32.8;     //full scale accelerometer range (32.8 for 500 degrees/sec) - reference dRehmFlight, using 1 for now to see raw IMU output
float AccX_prev, AccY_prev, AccZ_prev, GyroX_prev, GyroY_prev, GyroZ_prev;
float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)

//servo info
#define SERVO_CENTER_DEGREES 90
#define SERVO_TURN_RIGHT_SOFT_DEGREES 50
#define SERVO_TURN_RIGHT_HARD_DEGREES 30
#define SERVO_TURN_LEFT_SOFT_DEGREES 130
#define SERVO_TURN_LEFT_HARD_DEGREES 150

//timing info
#define STRAIGHT_POST_RELEASE_MILLIS 2000
#define TURNAROUND_MILLIS 5000
#define STRAIGHT_POST_TURNAROUND_MILLIS 2000
//will be used as "stopwatches" to achieve the timings outlined above for each stage of the flight
int flightStateTimingStart, flightStateTimingEnd;

/* Tasks */
#define GPS_TASK_RATE 250
#define IMU_TASK_RATE 250
#define STROBE_TASK_RATE 250
#define SD_TASK_RATE 1000
Task gpsTask(GPS_TASK_RATE, TASK_FOREVER, &updateGPS);      //update GPS global variables every GPS_TASK_RATE ms, will be enabled from the start
Task imuTask(IMU_TASK_RATE, TASK_FOREVER, &updateIMU);      //update IMU global variables every IMU_TASK_RATE ms, will be enabled from the start
Task strobeTask(STROBE_TASK_RATE, TASK_FOREVER, &strobe);   //strobe lights every STROBE_TASK_RATE ms, won't be enabled until release
Task sdTask(SD_TASK_RATE, TASK_FOREVER, &outputToSD);

/* Declaring Components */
Scheduler runner;                     //declare our task scheduler
MPU6050 mpu6050;                      //declare MPU6050 object for our IMU
TinyGPSPlus gps;                      //declare TinyGPSPlus object to handle GPS data (convert raw data to be more user friendly)
SoftwareSerial ss_GPS(RXPin, TXPin);  //declare serial connection to the GPS module
SensorDataStruct sensorData;          //declare struct to hold our sensor data
PWMServo rudderServo;                 //declare servo to control rudder

void setup() {
  //setup Serial communication
  Serial.begin(MONITOR_BAUD);
  ss_GPS.begin(GPS_BAUD);

  //setup pins
  //note: RX and TX pins are set as input and output by default, don't need to explicitly set them here
  pinMode(LEDPin, OUTPUT);
  pinMode(LimitPin, INPUT_PULLUP);
  pinMode(ServoPin, OUTPUT);

  //setup servo
  rudderServo.attach(ServoPin); //attach pin to servo object
  moveRudder(SERVO_CENTER_DEGREES);               //set servo to center position

  //setup IMU
  IMUinit();

  //initialize SD card
  if (SD.begin(CS)) {
    Serial.println("SD card initialized");
  }
  else {
    Serial.println("SD card initialization failed");
  }

  //setup runners and tasks
  runner.init();
  Serial.println("Initialized scheduler");
  
  runner.addTask(gpsTask);
  Serial.println("added gpsTask");
  
  runner.addTask(imuTask);
  Serial.println("added imuTask");

  runner.addTask(strobeTask);
  Serial.println("added strobeTask");

  runner.addTask(sdTask);
  Serial.println("added sdTask");

  delay(1000);
  
  gpsTask.enable();
  Serial.println("Enabled gpsTask");
  imuTask.enable();
  Serial.println("Enabled imuTask");
  sdTask.enable();
  Serial.println("Enabled sdTask");

  delay(1000);

  //uncomment to calculate IMU error values 
  //calculate_IMU_error();
  //delay(1000);
}

void loop() {
  runner.execute();
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
            strobeTask.enable();
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
      }
      flightState = DESCENT_CIRCLE;
      break;
    case DESCENT_CIRCLE:
      Serial.println("descent circle");
      //set rudder to circle in scoring zone until landing
      moveRudder(SERVO_TURN_LEFT_HARD_DEGREES); //CAN CHANGE TO TURN RIGHT
      while(1){
        //infinite loop, circle until landing
      }
      break;
    case ERROR_STATE:
      Serial.println("In ERROR flight state!!!");
      break;
  }
}

void updateGPS() {
  //Serial.println("updateGPS");
  // get GPS data from i2C
  while(ss_GPS.available() > 0){
    gps.encode(ss_GPS.read());
    if(gps.location.isUpdated()){
      //if the received gps data is successfully encoded, update global variables
      sensorData.gpsLat = gps.location.lat();
      sensorData.gpsLng = gps.location.lng();
      
      // //for testing
      // char buffer [50];
      // sprintf(buffer, "Latitude: %f, Longitude: %f", sensorData.gpsLat, sensorData.gpsLng);
      // Serial.println(buffer);
    }
  }
}

void updateIMU() {
  //Serial.println("updateIMU");

  // Get new raw IMU values
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
  mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

  // Correct and scale accel and gyro values
  float tempAcX = (AcX / ACCEL_SCALE_FACTOR) - AccErrorX;
  float tempAcY = (AcY / ACCEL_SCALE_FACTOR) - AccErrorY;
  float tempAcZ = (AcZ / ACCEL_SCALE_FACTOR) - AccErrorZ;
  float tempGyX = (GyX / GYRO_SCALE_FACTOR) - GyroErrorX;
  float tempGyY = (GyY / GYRO_SCALE_FACTOR) - GyroErrorY;
  float tempGyZ = (GyZ / GYRO_SCALE_FACTOR) - GyroErrorZ;

  // LP filter scaled values and save them to global vars
  sensorData.accelX = (1.0 - B_accel)*AccX_prev + B_accel*tempAcX;
  sensorData.accelY = (1.0 - B_accel)*AccY_prev + B_accel*tempAcY;
  sensorData.accelZ = (1.0 - B_accel)*AccZ_prev + B_accel*tempAcZ;
  sensorData.roll   = (1.0 - B_gyro)*GyroX_prev + B_gyro*tempGyX;
  sensorData.pitch  = (1.0 - B_gyro)*GyroY_prev + B_gyro*tempGyY;
  sensorData.yaw    = (1.0 - B_gyro)*GyroZ_prev + B_gyro*tempGyZ;

  // Update previous variables
  AccX_prev = sensorData.accelX;
  AccY_prev = sensorData.accelY;
  AccZ_prev = sensorData.accelZ;
  GyroX_prev = sensorData.roll;
  GyroY_prev = sensorData.pitch;
  GyroZ_prev = sensorData.yaw;
}

void IMUinit(){
  //initialize the IMU
  Wire.begin();
  Wire.setClock(200000); //400 kHz max...

  mpu6050.initialize();

  if (mpu6050.testConnection() == false) {
    Serial.println("MPU6050 initialization unsuccessful");
    Serial.println("Check MPU6050 wiring or try cycling power");
    while(1) {}
  }

  //research the details of these ranges (taken from dRehmFlight)
  mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu6050.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

}

void calculate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  //Credit: dRehmFlight - https://github.com/nickrehm/dRehmFlight/blob/master/Versions/dRehmFlight_Teensy_BETA_1.3/dRehmFlight_Teensy_BETA_1.3.ino
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement. 
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;  //temp vars for getting raw values from the IMU
  float AccX, AccY, AccZ, GyroX, GyroY, GyroZ; //temp vars for scaling IMU values
  
  AccErrorX = 0.0;
  AccErrorY = 0.0;
  AccErrorZ = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY= 0.0;
  GyroErrorZ = 0.0;
  
  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    
    AccX  = AcX / ACCEL_SCALE_FACTOR;
    AccY  = AcY / ACCEL_SCALE_FACTOR;
    AccZ  = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
    
    //Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");
  
  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}

void strobe() {
  Serial.println("Strobe");

  // Stobe LEDs by setting them to HIGH if they were LOW, and LOW if they were HIGH
  if(digitalRead(LEDPin) == LOW){
    digitalWrite(LEDPin, HIGH);
  } else {
    digitalWrite(LEDPin, LOW);
  }
}

void moveRudder(int angle){
  Serial.print("Moving rudder to ");
  Serial.print(angle);
  Serial.println(" degrees.");

  //correct angle if it is an invalid value
  int rudderLowerBound = 90 - (RUDDER_RANGE_DEGREES / 2);
  int rudderUpperBound = 90 + (RUDDER_RANGE_DEGREES / 2);
  if(angle < rudderLowerBound) {
    angle = rudderLowerBound;
  } else if (angle > rudderUpperBound) {
    angle = rudderUpperBound;
  }

  //set servo position
  rudderServo.write(angle);
}

void outputToSD(){
  // open the file named FILENAME on the SD card
  File dataFile = SD.open(FILENAME, FILE_WRITE);
  // if the file is available, write the contents of datastring to it
  if (dataFile) {
    char buffer[100];
    sprintf(buffer, "%f %f %f %f %f %f %f %f %d %d %d", sensorData.gpsLat, 
              sensorData.gpsLng, sensorData.roll, sensorData.pitch, sensorData.yaw, 
              sensorData.accelX, sensorData.accelY, sensorData.accelZ, digitalRead(LimitPin), rudderServo.read(), flightState);
    dataFile.println(buffer);
    Serial.println(buffer);
    //closing the file often flushes the data to the SD often, makes sure it actually gets saved on power down
    dataFile.close();
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening file");
  }  
}

