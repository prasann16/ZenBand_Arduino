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


// Firebase setup
#define FIREBASE_HOST "zen-band.firebaseio.com"
#define FIREBASE_AUTH "AcLQKRErBfKH4xnT3NHcHTeR1WfAx4lbLyN2yIBi"

// Wifi connection setup
//#define WIFI_SSID "SM-G935W89532"
//#define WIFI_PASSWORD "esp8266-test"
#define WIFI_SSID "VIRGIN535"
#define WIFI_PASSWORD "5A4FC75E"

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

int HR_thres;
int GSR_thres;
int Temp_thres;
int Acc_thres;
int anxiety_level;
int activity;

void setup() {
  //.................................PPG Sensor...............................................
// Serial port initialization
  Serial.begin(115200);
  Serial.println("Initializing...");
  delay(10);

// setting all digital pins as outputs
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(1, OUTPUT);

  //  Setting all digital pins to low to save power
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(14, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  digitalWrite(15, LOW);
  digitalWrite(3, LOW);
  digitalWrite(1, LOW);

//Connecting to Wifi
//  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
//  Serial.print("connecting");
//  while (WiFi.status() != WL_CONNECTED){
//    Serial.print(".");
//    delay(500);
//  }
//  Serial.println();
//  Serial.print("connected: ");
//  Serial.println(WiFi.localIP());
//
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

// Calibrating the device and finding thresholds

  delay(5000);
  int HR_sum = 0;
  int GSR_sum = 0;
  int Temp_sum = 0;
  int Acc_sum = 0;
  int prev_Acc = 0;
  int i=0; // Loop counter
  int Temp_counter = 0;
  int GSR_counter = 0;
  int start_time = millis();

  Serial.println("Calibrating................")
  
  while((millis()-start_time)<15000){
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
    
    if(i==0){
      prev_Acc = AcX+AcY+AcZ;
    }
    else{
      Acc_sum = Acc_sum + (AcX+AcY+AcZ - prev_Acc);
    }
    //  ......................................................Temprature Sensor Calculations...........................................................
    if(i%2 == 0) {
      digitalWrite(4, LOW);
      Vo = analogRead(analog);
      R2 = R1 * (1023.0 / (float)Vo - 1.0);
      logR2 = log(R2);
      T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
      Tc = T - 273.15;
      Temp_sum = Temp_sum + Tc;
      Temp_counter = Temp_counter + 1;
    }
    //  ......................................................GSR Sensor Calculations...........................................................
    else{
      digitalWrite(4, HIGH);
      GSR_sum = GSR_sum + analogRead(analog);
      GSR_counter = GSR_counter +1; 
    }

    HR_sum = HR_sum + beatAvg;
//    Updating the loop
    i = i+1;
  }

  int HR_thres = HR_sum/i;
  int GSR_thres = GSR_sum/GSR_counter;
  int Temp_thres = Temp_sum/Temp_counter;
  int Acc_thres = Acc_sum/(i-1);

  Serial.print("Baseline Heart Rate: ");
  Serial.print(HR_thres);
  Serial.print("Baseline Skin Resistance: ");
  Serial.print(GSR_thres);
  Serial.print("Baseline Skin Temperature: ");
  Serial.print(Temp_thres);
  Serial.println();
  
  

//  Connect to Firebase
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}


void loop() {

  int loop_start = millis();
  int HR_sum = 0;
  int GSR_sum = 0;
  int Temp_sum = 0;
  int Acc_sum = 0;
  int prev_Acc = 0;
  int i=0; // Loop counter
  int Temp_counter = 0;
  int GSR_counter = 0;
  int anxiety_level = 0;
  int activity = 0;
  
  
 

//  To collect data for 3 sec and then send data to Firebase
  while((millis()-loop_start)<3000){

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

    if(i==0){
      prev_Acc = AcX+AcY+AcZ;
    }
    else{
      Acc_sum = Acc_sum + (AcX+AcY+AcZ - prev_Acc);
    }

    //  ......................................................Temprature Sensor Calculations...........................................................
    if(i%2 == 0) {
      digitalWrite(4, LOW);
      Vo = analogRead(analog);
      R2 = R1 * (1023.0 / (float)Vo - 1.0);
      logR2 = log(R2);
      T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
      Tc = T - 273.15;
      Temp_sum = Temp_sum + Tc;
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


  int HR_val = HR_sum/i;
  int GSR_val = GSR_sum/GSR_counter;
  int Temp_val = Temp_sum/Temp_counter;

  if((Acc_sum/(i-1))*1.2 > Acc_thres){
    activity = 1;
  }
  
  if(HR_val>(1.1*HR_thres) && GSR_val<(0.9*GSR_thres) && Temp_val<(0.9*Temp_thres) && activity==0){
    anxiety_level = 1;
  }
  else if(HR_val>(1.2*HR_thres) && GSR_val<(0.8*GSR_thres) && Temp_val<(0.85*Temp_thres) && activity==0){
    anxiety_level =2;
  }
  
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["Time"] = millis()/1000;
  root["Skin temperature"] = Temp_sum/Temp_counter;
  root["Galvanic Skin Response"] = GSR_sum/GSR_counter;
//  IF HR value has error
  if((HR_sum/i) < 50){
    root["Heart Rate"] = 0;
  }
  else{
    root["Heart Rate"] = HR_sum/i;
  }
  
  root["Activity"] = activity;
  root["Anxiety Level"] = anxiety_level;
  root["HR threshold"] = HR_thres;
  root["GSR threshold"] = GSR_thres;
  root["Temp threshold"] = Temp_thres;
  Firebase.push("Values", root);
  
}
