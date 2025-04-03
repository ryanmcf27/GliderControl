/*
  Combined_Test

  Code to test all of the functionality of Iowa State University's 2024-2025 Design Build Fly (DBF) team's X-1 glider
  
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
void servoResponse();
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
  float gyroX = 0.0;      //X-1's deg/sec around the X  axis 
  float gyroY = 0.0;      //X-1's deg/sec around the Y  axis 
  float gyroZ = 0.0;      //X-1's deg/sec around the Z  axis 
  float accelX = 0.0;     //X-1's acceleration in the X direction
  float accelY = 0.0;     //X-1's acceleration in the Y direction
  float accelZ = 0.0;     //X-1's acceleration in the Z direction
};

/* Constants and Pins */
static const int RXPin = 0, TXPin = 1, ServoPin = 3, LEDPin = 5, LimitPin = 8;
static const uint32_t MONITOR_BAUD = 115200, GPS_BAUD = 9600;
static const int RUDDER_RANGE_DEGREES = 180; //Limits the servo's movement from (90 - RUDDER_RANGE_DEGREES) to (90 + RUDDER_RANGE_DEGREES), corrects if invalid input is given to moveRudder function

//values for storing data to the SD card
const int CS = BUILTIN_SDCARD;
const char* FILENAME = "Combined_Test_OUT.txt";


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


/* Tasks */
#define SERVO_TASK_RATE 1000
#define IMU_TASK_RATE 100
#define STROBE_TASK_RATE 250
#define SD_TASK_RATE 500
Task servoTask(SERVO_TASK_RATE, TASK_FOREVER, &servoResponse);      //update servo position based on limit switch, will be enabled from the start
Task imuTask(IMU_TASK_RATE, TASK_FOREVER, &updateIMU);              //update IMU global variables every IMU_TASK_RATE ms, will be enabled from the start
Task strobeTask(STROBE_TASK_RATE, TASK_FOREVER, &strobe);           //strobe lights every STROBE_TASK_RATE ms, won't be enabled until release
Task sdTask(SD_TASK_RATE, TASK_FOREVER, &outputToSD);               //store sensor data to SD card and print it to serial monitor

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
  moveRudder(90);               //set servo to 90 degrees

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
  
  runner.addTask(servoTask);
  Serial.println("added servoTask");
  
  runner.addTask(imuTask);
  Serial.println("added imuTask");

  runner.addTask(strobeTask);
  Serial.println("added strobeTask");

  runner.addTask(sdTask);
  Serial.println("added sdTask");

  delay(1000);
  
  //enable all tasks for testing
  servoTask.enable();
  Serial.println("Enabled servoTask");
  imuTask.enable();
  Serial.println("Enabled imuTask");
  // strobeTask.enable();
  // Serial.println("Enabled strobeTask");
  sdTask.enable();
  Serial.println("Enabled sdTask");

  delay(1000);

  //uncomment to calculate IMU error values 
  //calculate_IMU_error();
  //delay(1000);
}

// test can analyze the functionality of all of the sensors based on the output to the SD care and terminal (from sdTask)
// may want to increase the task rate of sdTask depending on how sensitive you would like the test to be
void loop() {
  // TaskScheduler manages most of the tasks
  runner.execute();
  // GPS handled seperately due to delay issues interferring with reading I2C data
  updateGPS();
}

void servoResponse() {
  //move the servo to a position based on whether the limit switch is pressed or not
  if(digitalRead(LimitPin)){
    moveRudder(30);
  } else {
    moveRudder(150);
  }
}

void updateGPS() {
  // get GPS data from i2C
  while(ss_GPS.available() > 0){
    gps.encode(ss_GPS.read());
    if(gps.location.isUpdated()){
      //if the received gps data is successfully encoded, update global variables
      sensorData.gpsLat = gps.location.lat();
      sensorData.gpsLng = gps.location.lng();
    }
  }
}

void updateIMU() {
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
  sensorData.gyroX   = (1.0 - B_gyro)*GyroX_prev + B_gyro*tempGyX;
  sensorData.gyroY  = (1.0 - B_gyro)*GyroY_prev + B_gyro*tempGyY;
  sensorData.gyroZ    = (1.0 - B_gyro)*GyroZ_prev + B_gyro*tempGyZ;

  // Update previous variables
  AccX_prev = sensorData.accelX;
  AccY_prev = sensorData.accelY;
  AccZ_prev = sensorData.accelZ;
  GyroX_prev = sensorData.gyroX;
  GyroY_prev = sensorData.gyroY;
  GyroZ_prev = sensorData.gyroZ;
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
  // Stobe LEDs by setting them to HIGH if they were LOW, and LOW if they were HIGH
  if(digitalRead(LEDPin) == LOW){
    digitalWrite(LEDPin, HIGH);
  } else {
    digitalWrite(LEDPin, LOW);
  }
}

void moveRudder(int angle){
  // Serial.print("Moving rudder to ");
  // Serial.print(angle);
  // Serial.println(" degrees.");

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
  // // open the file named FILENAME on the SD card
  // File dataFile = SD.open(FILENAME, FILE_WRITE);
  // // if the file is available, write the contents of datastring to it
  // if (dataFile) {
  //   char buffer[100];
  //   sprintf(buffer, "%f %f %f %f %f %f %f %f %d %d", sensorData.gpsLat, 
  //             sensorData.gpsLng, sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ, 
  //             sensorData.accelX, sensorData.accelY, sensorData.accelZ, digitalRead(LimitPin), rudderServo.read());
  //   dataFile.println(buffer);
  //   Serial.println(buffer);
  //   //closing the file often flushes the data to the SD often, makes sure it actually gets saved on power down
  //   dataFile.close();
  // }  
  // // if the file isn't open, pop up an error:
  // else {
  //   Serial.println("error opening file");
  // }  

  char buffer[100];
    sprintf(buffer, "%f %f %f %f %f %f %f %f %d %d", sensorData.gpsLat, 
              sensorData.gpsLng, sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ, 
              sensorData.accelX, sensorData.accelY, sensorData.accelZ, digitalRead(LimitPin), rudderServo.read());
    
    Serial.println(buffer);
}
