//Logs only OPC-R1 and HYT
//needs OPC-R1_BL4 on the OPC-Arduino

#include <Wire.h>
#include <SoftEasyTransfer.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include "RTClib.h"
#define HYT939_ADDR 0x28
#define TFACTOR 99.2909
#define TDELTA 40.0
#define HFACTOR 163.83
#define dryerPin 4
#define dryerMinT 34
#define dryerMaxT 35

bool dryerOn = false;
bool overtemp = false;

//Adafruit_ADS1115 ads(0x48);


RTC_PCF8523 rtc;
//RTC_DS1307 rtc;
DateTime now1;

char fileName[15] = "datalog.txt";
File myFile;

SoftwareSerial opcSerial(6, 7); //OPC rx, tx

//create object
SoftEasyTransfer ETopc; 

struct RECEIVE_DATA_STRUCTURE {
  unsigned int hist[16];
  float temp;  
  float humid;
  float PM[3];
};
RECEIVE_DATA_STRUCTURE mydata;

//String timeStr;

void setup(){
  pinMode(dryerPin, OUTPUT);  
  Wire.begin();
  pinMode(10, OUTPUT); // SD Card CS

  if (! rtc.initialized()) {  //PCF8523
  //if (! rtc.isrunning()) {    //DS1307
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    //rtc.adjust(DateTime(2017, 1, 21, 3, 0, 0));
  }
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
 
  opcSerial.begin(9600);
  Serial.begin(9600);
  //start the library, pass in the data details and the name of the serial port.
  ETopc.begin(details(mydata), &opcSerial);

  if (!SD.begin(10)) {
    Serial.println("SDcard not ready"); 
    delay(10000);
  }
  else
    Serial.println("SDcard ok"); 
  
  if (!SD.exists(fileName)) {
    myFile = SD.open(fileName, FILE_WRITE);
    myFile.println("###");
    myFile.flush();
  }
  else {
    myFile = SD.open(fileName, FILE_WRITE);
    myFile.println("-----");
    myFile.flush();
  }  


  Serial.println("Setup done, now logging...");  
}

void loop(){
  
    if(ETopc.receiveData()){   //opc received 
      now1 = rtc.now();  

      
      Serial.print(now1.day());
      Serial.print('.');
      Serial.print(now1.month());
      Serial.print('.');
      Serial.print(now1.year()); 
      Serial.print(' ');
      Serial.print(now1.hour());
      Serial.print(':');
      Serial.print(now1.minute());
      Serial.print(':');
      Serial.print(now1.second()); 
      Serial.print(' ');
      
      
      Serial.print(';');
      for (int i = 0; i<16; i++) {
        Serial.print(mydata.hist[i]);
        Serial.print(' ');
      }    
      Serial.print(';');
      Serial.print(mydata.temp);
      Serial.print(' ');
      Serial.print(mydata.humid);
      Serial.print(';');
      for (int i = 0; i<3; i++) {
        Serial.print(mydata.PM[i]);
        Serial.print(' ');
      }    
      Serial.print(';');
      
      //myFile.print(timeStr);
      // print time without String

      DateTime now = rtc.now();
      myFile.print(now.day(), DEC);
      myFile.print('.');
      myFile.print(now.month(), DEC);
      myFile.print('.');
      myFile.print(now.year(), DEC);
      myFile.print(' ');
      myFile.print(now.hour(), DEC);
      myFile.print(':');
      myFile.print(now.minute(), DEC);
      myFile.print(':');
      myFile.print(now.second(), DEC);
      



      
      myFile.print('\t');
      for (int i = 0; i<16; i++) {
        myFile.print(mydata.hist[i]);
        myFile.print('\t');
      }    
      myFile.print(mydata.temp);
      myFile.print('\t');
      myFile.print(mydata.humid);
      myFile.print('\t');
      for (int i = 0; i<3; i++) {
        myFile.print(mydata.PM[i]);
        myFile.print('\t');
      }  

      //HYT
      unsigned int traw;
      unsigned int hraw;
      
      double temp;
      double hum;
      int i;
      unsigned char buffer[4];
  
      Wire.beginTransmission(HYT939_ADDR); // transmit to device #44 (0x2c)
      Wire.endTransmission(); // stop transmitting    
      //100ms warten
      delay(100);
      //4 Bytes vom Sensor lesen
      Wire.requestFrom(HYT939_ADDR, 4,true);
      i=0;
      while(Wire.available()) {
        char c = Wire.read(); // receive a byte as character
        buffer[i]=c;
        i++;
      }
      //Rohdaten aus Puffer lesen
      traw=buffer[2]*256+buffer[3];
      hraw=buffer[0]*256+buffer[1];
      //Daten laut Datenblatt maskieren
      traw&=0xfffc;
      hraw&=0x3fff;
      traw=traw/4;
      //Rohdaten Umrechnen
      temp=(double)traw/TFACTOR;
      temp=temp-TDELTA;
      hum=(double)hraw/HFACTOR;
      
      Serial.print("Temp:");
      Serial.print(temp);
      Serial.print(' ');
      Serial.print("Hum:");
      Serial.print(hum);
      Serial.print(';');

  
      myFile.print(temp);
      myFile.print('\t');
      myFile.print(hum);
      myFile.print('\t');

      //dryer control
      if (hum > 65) {
        if (mydata.temp < dryerMinT) {
          dryerOn = true;
          overtemp = false;
        }
        else {
          if (mydata.temp > dryerMaxT) {
            dryerOn = false;
            overtemp = true;
          }  
          else { // between dryerMinT and dryerMaxT
            if (overtemp) {
              dryerOn = false; //let it cool below dryerMinT
            }
            else {
              dryerOn = true; //let it reach dryerMaxT
            }   
          }
        }
      }
      else {
        dryerOn = false;
      }
      
      if (dryerOn) {
          digitalWrite(dryerPin,HIGH);
          Serial.println('1');
          myFile.print('1');       
      }
      else {
          digitalWrite(dryerPin,LOW);
          Serial.println('0');
          myFile.print('0');
      }


        
      myFile.println();
      myFile.flush();  

      delay(2000);
    }    
}
