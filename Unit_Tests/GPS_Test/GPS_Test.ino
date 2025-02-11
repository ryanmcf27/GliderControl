#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

/* Method Prototypes */
void updateGPS();

static const int RXPin = 0, TXPin = 1;
int i = 0;

TinyGPSPlus gps;                      //declare TinyGPSPlus object to handle GPS data (convert raw data to be more user friendly)
SoftwareSerial ss_GPS(RXPin, TXPin);  //declare serial connection to the GPS module 

void setup() {
  Serial.begin(115200); //115200
  ss_GPS.begin(9600);
}

void loop() {
  updateGPS();
  delay(4000);
  Serial.println();
  //i++;


  // while(ss_GPS.available()){
  //   byte gpsData = ss_GPS.read();
  //   Serial.write(gpsData);
  // }
}

void updateGPS() {
  //Serial.println("Updating GPS...");
  // get GPS data from i2C
  while(ss_GPS.available()){
    byte gpsData = ss_GPS.read();
    Serial.write(gpsData);
  }
  // if (ss_GPS.available()){
  //   //Serial.println(ss_GPS.read());
  //   byte gpsData = ss_GPS.read();
  //   Serial.write(gpsData);
  //   //gps.encode(ss_GPS.read());
  //   //if(gps.location.isUpdated()){
  //     //if the received gps data is successfully encoded, print data
  //     //Serial.print("gpsLat: ");
  //     //Serial.println(gps.location.lat());
  //     //Serial.print("gpsLng: ");
  //     //Serial.println(gps.location.lng());
  //   //} else {
  //   //  Serial.println("not updated");
  //   //}
  // } else {
  //   Serial.println("not available");
  // }
}