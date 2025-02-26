//Heavily based on example code by Taylor Schweizer - https://forum.arduino.cc/t/i2c-protocol-tutorial-using-an-mpu6050/387512

#include "Wire.h"  // This library allows you to communicate with I2C devices.

void calculate_IMU_error();

const int MPU_addr = 0x68;  // I2C address of the MPU-6050

float AccX_prev, AccY_prev, AccZ_prev;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float B_accel = 0.5;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = -0.08;
float AccErrorY = 0.01;
float AccErrorZ = -0.02;
float GyroErrorX = -1.52;
float GyroErrorY= 0.86;
float GyroErrorZ = 0.79;

void setup() {
  Serial.begin(38400);
  //begin I2C communication
  Wire.begin();
  Wire.beginTransmission(MPU_addr);  // Begins a transmission to the I2C slave (the MPU-6050)
  //First, tell MPU which register to write to (PWR_MGM_1 is located at 0x6B)
  Wire.write(0x6B);                  // PWR_MGMT_1 register
  Wire.write(0);                     // set to zero (wakes up the MPU-6050)
  //temporarily end transmission to send to new register address
  Wire.endTransmission(true);       
  delay(50);
  Wire.beginTransmission(MPU_addr);

  /* Configure Gyroscope */
  //Here is a breakdown of the bits:
  //Bit0 - Reserved, set to 0
  //Bit1 - Reserved, set to 0
  //Bit2 - Reserved, set to 0
  //Bit3 - FS_SEL
  //Bit4 - FS_SEL  -  This sets the full scale range of the gyroscope. We want to set FS_SEL to the +-500°/s setting, which is decimal 1, hex 0x01, binary 1
  //Bit5 - ZG_ST - Activates self test of gyroscope Z axis if set to 1. We will set it to 0
  //Bit6 - YG_ST - same as ZG_ST but with Y axis
  //Bit7 - XG_ST - same as ZG_ST but with X axis

  //According to those values, we want to send the value 0 0 0 0 1 0 0 0, which is dec 8 hex 0x08
  //And remember, we want to send it to 0x1B
  Wire.write(0x1B);   // send to GYRO_CONFIG (register 0x1B)
  Wire.write(0x08);   // data to send

  //End and restart transmission
  Wire.endTransmission(true);
  delay(50);
  Wire.beginTransmission(MPU_addr);

  /* Configure Accelerometer */
  //I'll skip going through figuring out each bit, and just say I want to send 0x00 - full scale range can be +-2g. The address is 0x1C - UPDATE with max g's
  Wire.write(0x1C);
  Wire.write(0x00);

  //Once we send the values, we end the transmission because we don't have any further data to send for now
  Wire.endTransmission(true);

  //uncomment to calibrate IMU error
  calculate_IMU_error();
}

void loop() {

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);
  int16_t XAxisFull =  (Wire.read() << 8 | Wire.read());
  int16_t YAxisFull =  (Wire.read() << 8 | Wire.read());
  int16_t ZAxisFull =  (Wire.read() << 8 | Wire.read());
  //correct LSB sensitivity based on full scale range (pg 29 of register map)
  float XAxisFinal = (float)XAxisFull / 16384.0;
  float YAxisFinal = (float)YAxisFull / 16384.0;
  float ZAxisFinal = (float) ZAxisFull / 16384.0;
  //LP Filter
  XAxisFinal = (1.0 - B_accel)*AccX_prev + B_accel*XAxisFinal;
  YAxisFinal = (1.0 - B_accel)*AccY_prev + B_accel*YAxisFinal;
  ZAxisFinal = (1.0 - B_accel)*AccZ_prev + B_accel*ZAxisFinal;
  AccX_prev = XAxisFinal;
  AccY_prev = YAxisFinal;
  AccZ_prev = ZAxisFinal;

  //Serial.printf("%f %f %f\n", XAxisFinal, YAxisFinal, ZAxisFinal);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);
  int16_t XGyroFull = Wire.read() << 8 | Wire.read();
  int16_t YGyroFull = Wire.read() << 8 | Wire.read();
  int16_t ZGyroFull = Wire.read() << 8 | Wire.read();
  float XGyroFinal = (float)XGyroFull/32.8;
  float YGyroFinal = (float)YGyroFull/32.8;
  float ZGyroFinal = (float)ZGyroFull/32.8;
  
  Serial.printf("%f %f %f %f %f %f\n", XAxisFinal, YAxisFinal, ZAxisFinal, XGyroFinal, YGyroFinal, ZGyroFinal);


  delay(250);
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