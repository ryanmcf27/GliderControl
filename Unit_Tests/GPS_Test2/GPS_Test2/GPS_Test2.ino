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
  while(ss_GPS.available() > 0){
    gps.encode(ss_GPS.read());
    if(gps.location.isUpdated()){
      Serial.print("gpsLat: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("gpsLng: ");
      Serial.println(gps.location.lng(), 6);
    }
  }
}
