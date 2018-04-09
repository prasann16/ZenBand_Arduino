//
// Copyright 2015 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

// FirebaseDemo_ESP8266 is a sample that demo the different functions
// of the FirebaseArduino API.

#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include<Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Time.h>
#include "DataToMaker.h"
#include <StackArray.h>


// Firebase setup
#define FIREBASE_HOST "zen-band.firebaseio.com"
#define FIREBASE_AUTH "AcLQKRErBfKH4xnT3NHcHTeR1WfAx4lbLyN2yIBi"

// Wifi connection setup
#define WIFI_SSID "SM-G935W89532"
#define WIFI_PASSWORD "esp8266-test"
//#define WIFI_SSID "VIRGIN535"
//#define WIFI_PASSWORD "5A4FC75E"
//#define WIFI_SSID "iPhone"
//#define WIFI_PASSWORD "arhumRaza"
//#define WIFI_SSID "Rachel's iPhone"
//#define WIFI_PASSWORD "rachellim"
//#define WIFI_SSID "SM-G930W80587"
//#define WIFI_PASSWORD "abcdefgh"
//#define WIFI_SSIDf "dlink-4049"
//#define WIFI_PASSWORD "eidoa45979"
//#define WIFI_SSID "HTC"
//#define WIFI_PASSWORD "kennet1234"

//I2C pins
int D3 = 0; //SDA
int D4 = 2; //SCL

// PPG Sensor Variables initialization
MAX30105 particleSensor;

const byte RATE_SIZE = 10; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

//MPU-6050 Variables Initialization
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

const int analog=A0;
int Vo;
float R1 = 10000;
float logR2, R2, T, Tc, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;


//Threshold values

int HR_thres = 66;
int GSR_thres = 500;
float Temp_thres = 35.1;
int Acc_thres;
int anxiety_level;
int activity;
int prev_val = 0;
int HR_val;
int GSR_val;
float Temp_val;
int Acc_val;
int calibration_counter = 0;
int prev_GSR;
float prev_Temp;
int prev_HR;
int reset_counter;

//int probeHr;
//int probeGsr;
//int probeTemp;

//To send an sms
const char* myKey = "cIhGV2VKtEsz4LS3B5b3F3";
DataToMaker event(myKey, "Zenband_Anxiety_Trigger");

StackArray <int> HR_stack;
StackArray <int> GSR_stack;
StackArray <float> Temp_stack;


void setup() {
  //.................................PPG Sensor...............................................
// Serial port initialization
  Serial.begin(115200);
  Serial.println("Initializing...");
  delay(10);

// setting all digital pins as outputs
  pinMode(4, OUTPUT);

  //  Setting digital pin to low to save power
  digitalWrite(4, LOW);

//Connecting to Wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());

  // Initialize PPG sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");
//
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running

// Initialize MPU sensor
//  delay(10);
//  Wire.begin(0, 2);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
//  Connect to Firebase
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  
  root["Time"] = 0;
  root["Skin temperature"] = 0;
  root["Galvanic Skin Response"] = 0;
  root["Heart Rate"] = 0;
  root["Activity"] = 0;
  root["Anxiety Level"] = 0;
  root["HR threshold"] = 0;
  root["GSR threshold"] = 0;
  root["Temp threshold"] = 0;
  root["Sensor Reset"] = 1;
  
  Firebase.push("Values", root);

  HR_stack.push(HR_thres);
  GSR_stack.push(GSR_thres);
  Temp_stack.push(Temp_thres);
  prev_GSR = 500;
  prev_Temp = 35.1;
  prev_HR = 66;
  reset_counter = 0;

//  probeHr = 0;
//  probeGsr = 0;
//  probeTemp = 0;
}


void loop() {

  int HR_sum = 0;
  int GSR_sum = 0;
  float Temp_sum = 0;
  int Acc_sum = 0;
  int prev_Acc = 0;
  int prev_Ax = 0;
  int prev_Ay = 0;
  int prev_Az = 0;
  int i = 0; // Loop counter
  int Temp_counter = 0;
  int GSR_counter = 0;
  int anxiety_level = 0;
  int activity = 0;
  int Ax_sum = 0;
  int Ay_sum = 0;
  int Az_sum = 0;
  int loop_start = millis();
  

//  if(calibration_counter==0){
//    
//    
//  }
  
//  To collect data for 3 sec and then send data to Firebase
  while((millis()-loop_start)<2000){

    //  ......................................................PPG Sensor Stuff...........................................................
    long irValue = particleSensor.getIR();
    if (checkForBeat(irValue) == true)
    {
      // We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();
  
      beatsPerMinute = 60 / (delta / 1000.0);
  
      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable
  
        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
      if(irValue<5000){
        beatsPerMinute = 0;
        beatAvg = 0;
    } 

    HR_sum = HR_sum + beatAvg;
   
    //  ......................................................Accelerometer Sensor Stuff...........................................................
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

   if (i == 0) {
      prev_Acc = abs(AcX) + abs(AcY) + abs(AcZ);
      prev_Ax = abs(AcX);
      prev_Ay = abs(AcY);
      prev_Az = abs(AcZ);
    }
    else {
      Acc_sum = Acc_sum + (abs(AcX) + abs(AcY) + abs(AcZ) - prev_Acc);
      Ax_sum = Ax_sum+ abs(AcX) - prev_Ax ;
      Ay_sum = Ay_sum+ abs(AcY) - prev_Ay;
      Az_sum = Az_sum+ abs(AcZ) - prev_Az;
      
    }
    //  ......................................................Temprature Sensor Calculations...........................................................
    if(i%2 == 0) {
      digitalWrite(4, LOW);
      Vo = analogRead(analog);
      R2 = R1 * (1023.0 / (float)Vo - 1.0);
      logR2 = log(R2);
      T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
      Tc = T - 273.15;
      Temp_sum = float(Temp_sum) + float(Tc);
      Temp_counter = Temp_counter + 1;
    }
    //  ......................................................GSR Sensor Calculations...........................................................
    else{
      digitalWrite(4, HIGH);
      GSR_sum = GSR_sum + analogRead(analog);
      GSR_counter = GSR_counter +1; 
    }

//    Updating the loop
    i = i+1;
  }
//  Serial.print("Out of the loop");
  
//  Serial.println(HR_sum/i);
  HR_val = HR_sum/i;
  GSR_val = GSR_sum/GSR_counter;
  Temp_val = Temp_sum/Temp_counter;
  Acc_val = (Acc_sum / (i - 1));
  
//  if(GSR_val<100){
//    GSR_val = 512;
//  }

  Serial.print(HR_val);
  Serial.print("\t");
  Serial.print(GSR_val);
  Serial.print("\t");
  Serial.print(Temp_val);
  Serial.println();
  
// Detecting activity through accelerometer

   if (abs(Acc_val)>4000) {
    activity = 1;
    Serial.println("Performing activity.............................");
  }
  
//  Algorithm to detect anxiety
  
//  if (reset_counter == 1){
  if((HR_val>(1.05*prev_HR) || Temp_val<(0.98*prev_Temp) ) && activity==0){
    Serial.println("Anxiety Detected........................");
    if (event.connect()){
        Serial.println("Connected To SMS service");
        event.post();
    }
    
//    int HR_count_sum = 0;
//    int GSR_count_sum = 0;
//    float Temp_count_sum = 0;
//    int len = HR_stack.count();
//    
//    for(int a=0;a<len;a++){
//      HR_count_sum += HR_stack.pop();
//      GSR_count_sum += GSR_stack.pop();
//      Temp_count_sum += Temp_stack.pop();
//    }
//    HR_thres = HR_count_sum/len;
//    GSR_thres = GSR_count_sum/len;
//    Temp_thres = float(Temp_count_sum)/len;
    
    while(HR_val>(1.05*HR_thres) || (Temp_val<(0.98*Temp_thres))){
      int i = 0;
      int HR_sum = 0;
      int GSR_sum = 0;
      float Temp_sum = 0;
      int Temp_counter = 0;
      int GSR_counter = 0;
      int activity = 0;
      int start = millis();

      if(HR_val>(1.1*HR_thres) || (Temp_val<(0.95*Temp_thres))){
        anxiety_level = 2;
      }
      else if(HR_val>(1.05*HR_thres) || (Temp_val<(0.98*Temp_thres))){
        anxiety_level = 1;
      }
      
      while((millis()-start)<2000){

        //  ......................................................PPG Sensor Stuff...........................................................
        long irValue = particleSensor.getIR();
        if (checkForBeat(irValue) == true)
        {
          // We sensed a beat!
          long delta = millis() - lastBeat;
          lastBeat = millis();
      
          beatsPerMinute = 60 / (delta / 1000.0);
      
          if (beatsPerMinute < 255 && beatsPerMinute > 20)
          {
            rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
            rateSpot %= RATE_SIZE; //Wrap variable
      
            //Take average of readings
            beatAvg = 0;
            for (byte x = 0 ; x < RATE_SIZE ; x++)
              beatAvg += rates[x];
            beatAvg /= RATE_SIZE;
          }
        }
          if(irValue<5000){
            beatsPerMinute = 0;
            beatAvg = 0;
        } 
    
        HR_sum = HR_sum + beatAvg;
        //  ......................................................Temprature Sensor Calculations...........................................................
        if(i%2 == 0) {
          digitalWrite(4, LOW);
          Vo = analogRead(analog);
          R2 = R1 * (1023.0 / (float)Vo - 1.0);
          logR2 = log(R2);
          T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
          Tc = T - 273.15;
          Temp_sum = float(Temp_sum) + float(Tc);
          Temp_counter = Temp_counter + 1;
        }
        //  ......................................................GSR Sensor Calculations...........................................................
        else{
          digitalWrite(4, HIGH);
          GSR_sum = GSR_sum + analogRead(analog);
          GSR_counter = GSR_counter +1; 
        }
    
    //    Updating the loop
        i = i+1;
      }
      HR_val = HR_sum/i;
      GSR_val = GSR_sum/GSR_counter;
      Temp_val = Temp_sum/Temp_counter;

//      if(GSR_val<100){
//          GSR_val = 512;
//        }
      GSR_val = (Temp_val/prev_Temp)*(GSR_val)*0.8;

      Serial.print(HR_val);
      Serial.print("\t");
      Serial.print(GSR_val);
      Serial.print("\t");
      Serial.print(Temp_val);
      Serial.println();

      
      StaticJsonBuffer<200> jsonBuffer;
      JsonObject& root = jsonBuffer.createObject();

      root["Time"] = millis()/1000;
      root["Skin temperature"] = Temp_val;
      root["Galvanic Skin Response"] = GSR_val;
      root["Heart Rate"] = HR_val;
      root["Activity"] = 0;
      root["Anxiety Level"] = anxiety_level;
      root["HR threshold"] = HR_thres;
      root["GSR threshold"] = GSR_thres;
      root["Temp threshold"] = Temp_thres;
      root["Sensor Reset"] = 0;
      Firebase.push("Values", root);
    }
    
  }
  else{
    if(HR_stack.count()==5){
//      if(HR_val<1.1*prev_HR){
        HR_stack.pop();
        HR_stack.push(HR_val);
//      }
//      if(GSR_val>0.95*prev_GSR){
        GSR_stack.pop();
        GSR_stack.push(GSR_val);
//      }
//      if(Temp_val>0.95*prev_Temp){
        Temp_stack.pop();
        Temp_stack.push(Temp_val);
//      }
      
    }
    else{
      HR_stack.push(HR_val);
      GSR_stack.push(GSR_val);
      Temp_stack.push(Temp_val);
    }
  }
//  }
//  else{
//      HR_stack.push(HR_val);
//      GSR_stack.push(GSR_val);
//      Temp_stack.push(Temp_val);
//    }
//  else if(HR_val>(1.2*HR_thres) && GSR_val>(1.15*GSR_thres) && Temp_val<(0.95*Temp_thres) && activity==0){
//    anxiety_level = 2;
//    if (event.connect())
//      {
//        Serial.println("Connected To SMS service");
//        event.post();
//      }
//  } 
//  else if(HR_val>(1.1*HR_thres) && GSR_val>(1.1*GSR_thres) && Temp_val<(0.98*Temp_thres) && activity==0){
//    anxiety_level = 1;
//    if (event.connect())
//      {
//        Serial.println("Connected To SMS service");
//        event.post();
//      }
//  }
//  else if(GSR_val>(1.1*prev_GSR) && Temp_val<(0.98*prev_Temp) && HR_val>(1.1*prev_HR) && activity==0){
//    Serial.println("Anxiety Detected........................");
//    anxiety_level = 1;
//    if (event.connect())
//      {
//        Serial.println("Connected To SMS service");
//        event.post();
//      }
//  }

//    int HR_count_sum = 0;
//    int GSR_count_sum = 0;
//    float Temp_count_sum = 0;
//    int len = HR_stack.count();
//    
//    for(int a=0;a<len;a++){
//      HR_count_sum += HR_stack.pop();
//      GSR_count_sum += GSR_stack.pop();
//      Temp_count_sum += Temp_stack.pop();
//    }
//    HR_thres = HR_count_sum/len;
//    GSR_thres = GSR_count_sum/len;
//    Temp_thres = Temp_count_sum/len;

//Changing GSR proportional to temp
  GSR_val = (Temp_val/prev_Temp)*(GSR_val)*0.8;
  
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["Time"] = millis()/1000;
  root["Skin temperature"] = Temp_val;
  root["Galvanic Skin Response"] = GSR_val;
  root["Heart Rate"] = HR_val;
  root["Activity"] = activity;
  root["Anxiety Level"] = anxiety_level;
  root["HR threshold"] = HR_thres;
  root["GSR threshold"] = GSR_thres;
  root["Temp threshold"] = Temp_thres;
  root["Sensor Reset"] = 0;
  
  Firebase.push("Values", root);

//previous values of the loop

  prev_Temp = Temp_val;
  prev_GSR = GSR_val;
  prev_HR = HR_val;
//  reset_counter = 1;
  
}
