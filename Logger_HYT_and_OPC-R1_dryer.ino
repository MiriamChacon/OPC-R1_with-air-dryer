//Logs only OPC-R1 and HYT
//needs OPC-R1_BL4 on the OPC-Arduino

#include <Wire.h>
#include <SoftEasyTransfer.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
//#include <Adafruit_ADS1015.h>
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

  // setup adc
  //Serial.println("Getting single-ended readings from AIN0");
  //Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
  
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  // ADS1015  ADS1115  -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  //ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  //ads.begin();

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

      /*
      //read ADC
      int16_t adc_0, adc_1, adc_2, adc_3;
  
      adc_0 = ads.readADC_Differential_0_1();
      adc_1 = 0;
      adc_2 = ads.readADC_Differential_2_3();
      adc_3 = 0;
      //adc_0 = ads.readADC_SingleEnded(0);
      //adc_1 = ads.readADC_SingleEnded(1);
      //adc_2 = ads.readADC_SingleEnded(2);
      //adc_3 = ads.readADC_SingleEnded(3);
      Serial.print("Ch0: ");
      Serial.print(adc_0);
      //Serial.print(" Ch1:");
      //Serial.print(adc_1);
      Serial.print(" Ch2:");
      Serial.println(adc_2);
      //Serial.print(" Ch3:");
      //Serial.println(adc_3);

       
      
      
      //myFile.print('\t');
      myFile.print(adc_0);
      myFile.print('\t');
      //myFile.print(adc_1);
      //myFile.print('\t');
      myFile.print(adc_2);
      myFile.print('\t');
      //myFile.print(adc_3);
      //myFile.print('\t');
      */
  
      myFile.print(temp);
      myFile.print('\t');
      myFile.print(hum);
      myFile.print('\t');

      //dryer control
      if (hum > 70) {
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


