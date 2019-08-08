#include <Wire.h>
#include <LCD.h> // For LCD
#include <LiquidCrystal_I2C.h> // Added library*
#include "Adafruit_Sensor.h"
#include "Adafruit_L3GD20_U.h"
#include "Adafruit_LSM303_U.h"
#include "MadgwickAHRS.h"
#include <EEPROM.h>

//LCD""""""""""""""""""""""""""""""""""""""""""""""""""""
LiquidCrystal_I2C  lcd(0x38, 2, 1, 0, 4, 5, 6, 7); // 0x27 is the default I2C bus address of the backpack-see article
int ledPin = 0; 

//variables*********************************************
float declinationAngle = (1 + (49 / 60.0)); // Lausanne, here you have to write your own value.
int counter = 0,frequence;
float beginTimer = 0;
//buttons variables*****************************************
const int setButtonPin = 11;
const int plusButtonPin = 12;
const int minusButtonPin = 13;
const int buttonHigh = 9;
const int setButtonPinRadio = 1;
const int plusButtonPinRadio = 3;
const int minusButtonPinRadio = 2;
const int buttonHighRadio = 4;

int setButton = 0,plusButton = 0,minusButton = 0,setButtonRadio = 0,plusButtonRadio = 0,minusButtonRadio = 0;
long buttonTimer = 0;
long longPressTime = 2000;
boolean buttonActive = false;
boolean plusActive = false;
boolean minusActive = false;
boolean longPressActive = false;
int modeCalibHeading = 0;

int memmodeCalibHeading, memmaxindex, memKdHeading, memeKpHeading, memKGeneral,  memKiHeading, memdeadband,memdeadband_dHeading;
float memwantedHeading;
int buttontime;
float lastbutton = 0, delayTime = 0;

//Pid varibales**********************************************
float wantedpotentiometer = 512;
float heading = 0,wantedHeading = 1000, pHeading = 0, iHeading = 0,totError = 0;
float  KGeneral, KpHeading, KdHeading,KiHeading,maxindex = 4;
int reverse = 1; //1 for reverse operation
float previousError = 0, deltaError = 0,deltaErrorDeriv = 0,dHeading = 0, filteredDHeading = 0;
float cmd = 0.0;
unsigned long previousTime = 0, deltaTime, startMesure;
int deadband,deadband_dHeading;
float moyError;
int numMesure = 0;
float filteredPotError = 0;
float pourcentage = 0.5;
int motorSpeed;
float targetSpeed = 0, actualSpeed = 0, deltaSpeed = 0, minimalSpeed = 62;

//motor potentiometer****************************************
int PotentiometerPin = A0;    // select the input pin for the potentiometer
int PotentiometerPinPlus = A1;
int PotentiometerPinMinus = A2;
float Potentiometer, lastPotentiometer, potError = 0;
float potDeadband = 10;

// motor analog output***************************************
int RPWM=5;
int LPWM=6;
int L_EN=7;
int R_EN=8;

//sensor adress configuration****************************************
// Create sensor instances.
Adafruit_L3GD20_Unified       gyro(20);
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
float gyro_offsets[3]= {0,0,0};
float mag_offsets[3]            = { 3.96F, -21.47F, -2.51F };
float mag_softiron_matrix[3][3] = { { 1.081, -0.015, 0.039 },
                                    { 0.006, 1.074, -0.025 },
                                    { -0.005, 0.031, 1.099 } }; 
Madgwick filter;

//********************************************************************************************************************************************
//Buttons interface***************************************************************************************************************************
void interface(float orientation) {
  setButton = digitalRead(setButtonPin);
  plusButton = digitalRead(plusButtonPin);
  minusButton = digitalRead(minusButtonPin);
  //setButtonRadio = digitalRead(setButtonPinRadio);
  setButtonRadio = LOW;
  plusButtonRadio = digitalRead(plusButtonPinRadio);
  minusButtonRadio = digitalRead(minusButtonPinRadio);  
  if (setButton == HIGH or setButtonRadio == HIGH) {
    lastbutton=millis();
    if (buttonActive == false) {
      buttonActive = true;
      buttonTimer = millis();
    }
    if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) { //long press -> reset the motor position and stop the control
      longPressActive = true;
      wantedHeading = 1000;
      wantedpotentiometer = 512 ;
    }
  }
  else {
    if (buttonActive == true) {
      if (longPressActive == true) {
        longPressActive = false;
      } 
      else {
        if (wantedHeading==1000.0) {
          iHeading = 0;
          wantedHeading = orientation;
        }
        else {
          if (modeCalibHeading!=7) {
            modeCalibHeading+=1;
          }
          else {
            modeCalibHeading = 0; 
          }
          }
        }
      }
      buttonActive = false;
    }
  if (setButton == LOW and setButtonRadio == LOW and (plusButton == HIGH  or plusButtonRadio == HIGH) and millis()-lastbutton>500){ //and plusActive==false){
    lastbutton=millis();
      //if mode heading
      if (modeCalibHeading==0){
        if(wantedHeading<359){
          wantedHeading+= 1;
        }
        else {
          wantedHeading-=359;
        }
        }
      else if (modeCalibHeading==1){
        KGeneral += 1;
        EEPROM.write(0, (byte)KGeneral);
      }
      else if (modeCalibHeading==2){
        KpHeading += 1;
        EEPROM.write(1, (byte)KpHeading);
      }
      else if (modeCalibHeading==3){
        KdHeading += 1;
        EEPROM.write(2, (byte)KdHeading);
      }
      else if (modeCalibHeading==4){
        KiHeading += 5;
        EEPROM.write(3, (byte)KiHeading);
      }
      else if (modeCalibHeading==5){
        reverse =1;
        EEPROM.write(4, (byte)reverse);
      }
      else if (modeCalibHeading==6){
        deadband +=1;
        EEPROM.write(6, (byte)deadband);
      }
      else if (modeCalibHeading==7){
        deadband_dHeading +=1;
        EEPROM.write(7, (byte)deadband_dHeading);
      }
      plusActive = true;
  }
  else if (setButton == LOW and setButtonRadio == LOW and plusButton == LOW and plusButtonRadio == LOW and plusActive==true){
    plusActive = false;
  }
  if (setButton == LOW and setButtonRadio == LOW and (minusButton == HIGH or minusButtonRadio == HIGH) and millis()-lastbutton>500) {// and minusActive==false) {
    lastbutton=millis();
            //if mode heading
      if (modeCalibHeading==0){
        if(wantedHeading>1){
          wantedHeading-= 1;
        }
        else {
          wantedHeading+=359;
        }
      }
      else if (modeCalibHeading==1){
        KGeneral -= 1;
        EEPROM.write(0, (byte)KGeneral);
      }
      else if (modeCalibHeading==2){
        KpHeading -= 1;
        EEPROM.write(1, (byte)KpHeading);
      }
      else if (modeCalibHeading==3){
        KdHeading -= 1;
        EEPROM.write(2, (byte)KdHeading);
      }
      else if (modeCalibHeading==4){
        KiHeading -= 5;
        EEPROM.write(3, (byte)KiHeading);
      }
      else if (modeCalibHeading==5){
        reverse =0;
        EEPROM.write(4, (byte)reverse);
      }
      else if (modeCalibHeading==6){
        deadband -=1;
        EEPROM.write(6, (byte)deadband);
      }
      else if (modeCalibHeading==7){
        deadband_dHeading -=1;
        EEPROM.write(7, (byte)deadband_dHeading);
      }
      minusActive = true;
  }
  else if (setButton == LOW and setButtonRadio == LOW and plusButton == LOW and plusButtonRadio == LOW and minusActive==true){
    minusActive = false;
  }
}

//****************************************************************************
void led() {
  if (memKGeneral != KGeneral or memeKpHeading != KpHeading or memKdHeading != KdHeading or memwantedHeading != wantedHeading or memmodeCalibHeading != modeCalibHeading or memKiHeading != KiHeading or memdeadband != deadband or memdeadband_dHeading != deadband_dHeading) {
    buttontime = millis();    
  }
  if ((millis()-buttontime)<600000) {
    digitalWrite(ledPin,HIGH);
  }
  else {
    digitalWrite(ledPin,LOW);
  }
  memKGeneral = KGeneral;
  memeKpHeading = KpHeading;
  memKdHeading = KdHeading;
  memKiHeading = KiHeading;
  memwantedHeading = wantedHeading;
  memmodeCalibHeading = modeCalibHeading;
  memdeadband = deadband;
  memdeadband_dHeading = deadband_dHeading;
}

void setup() {
  // Set on LCD module
  pinMode(ledPin,OUTPUT);
  digitalWrite(ledPin,HIGH);
  lcd.begin (16, 2); // 16 x 2 LCD module
  lcd.clear();
  lcd.home (); // Set cursor to 0,0
  lcd.print("Initialisation...");

  //Serial Begin
  Serial.begin(115200);
  while(!Serial);
  Wire.begin();
  
  //Potentiometer motor
  pinMode(PotentiometerPin,INPUT);
  pinMode(PotentiometerPinPlus,OUTPUT);
  pinMode(PotentiometerPinMinus,OUTPUT);
  analogWrite(PotentiometerPinPlus,255);
  analogWrite(PotentiometerPinMinus,0);
  
  //buttons
  pinMode(buttonHigh,OUTPUT);
  digitalWrite(buttonHigh,HIGH);
  pinMode(setButtonPin, INPUT);
  pinMode(plusButtonPin, INPUT);
  pinMode(minusButtonPin, INPUT);
  pinMode(buttonHighRadio,OUTPUT);
  digitalWrite(buttonHighRadio,HIGH);
  pinMode(setButtonPinRadio, INPUT);
  pinMode(plusButtonPinRadio, INPUT);
  pinMode(minusButtonPinRadio, INPUT);
  //motor
  for(int i=5;i<9;i++){
    pinMode(i,OUTPUT);
  }
  for(int i=5;i<9;i++){
    digitalWrite(i,LOW);
  }
  digitalWrite(R_EN,HIGH);
  digitalWrite(L_EN,HIGH);
  
// Initialize the sensors.
  if(!gyro.begin()){
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
  if(!accel.begin()){
    Serial.println("Ooops, no L3M303DLHC accel detected ... Check your wiring!");
    while(1);
  }
  if(!mag.begin()){
    Serial.println("Ooops, no L3M303DLHC mag detected ... Check your wiring!");
    while(1);
  }
  sensors_event_t gyro_event;
  gyro.getEvent(&gyro_event);
  delay(1000); 
  gyro.getEvent(&gyro_event);
  delay(1000);
  for (int i=0;i<500;i++){
    sensors_event_t gyro_event;
    gyro.getEvent(&gyro_event);
    gyro_offsets[0]+=gyro_event.gyro.x/500;
    gyro_offsets[1]+=gyro_event.gyro.y/500;
    gyro_offsets[2]+=gyro_event.gyro.z/500;
  }

  //begin filter
  filter.begin((int)EEPROM.read(7));//frequence value

  /*//initial value write**************************************************************************************************
  EEPROM.write(0, (byte)10);//Kgeneral
  EEPROM.write(1, (byte)50);//KpHeading
  EEPROM.write(2, (byte)50);//KdHeading
  EEPROM.write(3, (byte)0);//KiHeading
  EEPROM.write(4, (byte)1);//reverse
  EEPROM.write(5, (byte)5);
  EEPROM.write(6, (byte)1);
  *///***********************************************************************************************************************
  
  //pid gains read
  KGeneral = (int)EEPROM.read(0);
  KpHeading = (int)EEPROM.read(1);
  KdHeading = (int)EEPROM.read(2);
  KiHeading = (int)EEPROM.read(3);
  reverse = (int)EEPROM.read(4);
  deadband = (int)EEPROM.read(5);
  deadband_dHeading = (int)EEPROM.read(6);
  
/*minimal speed calibration
  for (int i=0;i<255;i++) {
    Potentiometer = map(analogRead(PotentiometerPin), 630, 1023, 0, 1023);
    lastPotentiometer = Potentiometer;
    analogWrite(LPWM,i);
    analogWrite(RPWM,0);
    delay(10);
    Potentiometer = map(analogRead(PotentiometerPin), 630, 1023, 0, 1023);
    if (abs(Potentiometer-512)>10) {
      minimalSpeed = i;
      break;
    }
    else {
      lastPotentiometer = Potentiometer;
    }
  }
*/
  
  //lcd end of setup
  lcd.clear();
  lcd.home (); // Set cursor to 0,0
  lcd.print("Initialisation");
  lcd.setCursor (0, 1);       // Go to home of 2nd line
  lcd.print("Completed");
  delay(500);
  beginTimer = millis();
}

//********************************************************************************************
//********************************************************************************************
void loop() {
  
  if (millis()-beginTimer>2000 and millis()<20000) {
    frequence = counter/2;
    counter = 0;
    EEPROM.write(7, (byte)frequence);
    beginTimer = millis();
  }
  counter +=1;
  
  led();
  if (millis()-lastbutton>6000) {
    modeCalibHeading = 0;
  }
  Potentiometer = map(analogRead(PotentiometerPin), 630, 1023, 0, 1023);
  
  // Sensor
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  
  // Read accelerometer,Gyroscope,Magnetometer
  gyro.getEvent(&gyro_event);
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  
  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];
  
  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];
  
  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib)
  // where they are being converted)
  float gx = (gyro_event.gyro.x-gyro_offsets[0]) * 57.2958F;
  float gy = (gyro_event.gyro.y-gyro_offsets[1]) * 57.2958F;
  float gz = (gyro_event.gyro.z-gyro_offsets[2]) * 57.2958F;

  // Update the filter
  filter.update(gz, -gy, gx,
                accel_event.acceleration.z, -accel_event.acceleration.y, accel_event.acceleration.x,
                mz, -my, mx);

  // Print the orientation filter output
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float heading = filter.getYaw()+declinationAngle;
  
  /*
  //calibration*******************************************************
  Serial.flush(); 
  Serial.print(mz); 
  Serial.print(",");
  Serial.print(-my);
  Serial.print(",");
  Serial.print(mx);
  Serial.println();
  //*************************************
  */
  
  //PID Control*************************************************************************************************
  potError=Potentiometer-wantedpotentiometer;
  if (wantedHeading !=1000) {
    if (abs(heading - wantedHeading)>180) {
      if (heading > wantedHeading) {
        pHeading = (heading - wantedHeading)-360;
      }
      else {
        pHeading = (heading - wantedHeading)+360;
      }
    }
    else {
      pHeading = heading - wantedHeading;
    }
  }
  else {
    pHeading = 0;
  }
  if (pHeading>10) {
    pHeading = 10;
  }
  unsigned long now = millis();
  deltaTime = now - previousTime;
  previousTime = now;
  deltaError = pHeading - previousError;
  previousError = pHeading;
  dHeading = deltaError*1000/(deltaTime);
  filteredDHeading = (dHeading * (1 - pourcentage)) + (filteredDHeading * pourcentage);
  iHeading += pHeading*deltaTime/1000;
  if (iHeading>5120) {
    iHeading = 5120;
  }
  else if (iHeading<-5120) {
    iHeading = -5120;
  }
  if (abs(pHeading) < deadband/10) {
    pHeading = 0;
  }
  if (abs(dHeading) < deadband_dHeading/10) {
    dHeading = 0;
  }
  cmd = KGeneral/10 *( KpHeading * pHeading + KiHeading*iHeading + KdHeading*filteredDHeading);
  wantedpotentiometer = 512 - cmd; 
  if (wantedHeading == 1000.0){
    wantedpotentiometer = 512;
  }
  potError=Potentiometer-wantedpotentiometer;
  filteredPotError = (potError * (1 - pourcentage)) + (filteredPotError * pourcentage);
  motorSpeed = map(abs(512-wantedpotentiometer),0,512,0,160);
  if (motorSpeed>160) {
    motorSpeed = 160;
  }
  if (wantedHeading == 1000.0){
    motorSpeed = 25;
  }
  
  //Motor control*******************************************************************************
  if (filteredPotError>potDeadband) {
    targetSpeed = minimalSpeed+motorSpeed;
  }
  else if (filteredPotError<-potDeadband) {
    targetSpeed = -minimalSpeed-motorSpeed;
  }
  else if (filteredPotError<potDeadband and filteredPotError>-potDeadband) {
  targetSpeed = 0;
  }
  
  deltaSpeed = targetSpeed - actualSpeed;
  actualSpeed += deltaSpeed/6;
  
  if (actualSpeed>minimalSpeed) {
    analogWrite(LPWM,(1-reverse)*(actualSpeed));
    analogWrite(RPWM,reverse*(actualSpeed));
  }
  else if (actualSpeed<-minimalSpeed) {
    analogWrite(LPWM,reverse*(-actualSpeed));
    analogWrite(RPWM,(1-reverse)*(-actualSpeed));
  }
  else {
    analogWrite(LPWM,0);
    analogWrite(RPWM,0);
  }
   


//Display*************************************************************************************
  lcd.home (); // Set cursor to 0,0
  lcd.print("                ");
  lcd.home (); // Set cursor to 0,0
  lcd.print("Heading "); // Custom text
  lcd.print(heading,0);
  lcd.setCursor (0, 1);       // Go to home of 2nd line
  lcd.print("                ");
  lcd.setCursor (0, 1);
  if (modeCalibHeading==0){
    lcd.print("Wanted "); // Custom text
    lcd.print(wantedHeading,0);
  }
  else if (modeCalibHeading==1){
    lcd.print("KGeneral "); // Custom text
    lcd.print(KGeneral);
  }
  else if (modeCalibHeading==2){
    lcd.print("KpHeading "); // Custom text
    lcd.print(KpHeading);
  }
  else if (modeCalibHeading==3){
    lcd.print("KdHeading "); // Custom text
    lcd.print(KdHeading);
  }
  else if (modeCalibHeading==4){
    lcd.print("KiHeading "); // Custom text
    lcd.print(KiHeading);
  }
  else if (modeCalibHeading==5){
    lcd.print("Reverse "); // Custom text
    lcd.print(reverse);
  }
  else if (modeCalibHeading==6){
    lcd.print("Deadband "); // Custom text
    lcd.print(deadband);
  }
  else if (modeCalibHeading==7){
    lcd.print("Deadband_r "); // Custom text
    lcd.print(deadband_dHeading);
  }
  
  //Interface***************************************************************
  interface(heading);

  //Serial Info*************************************************************

  Serial.print("minimalSpeed ");
  Serial.print(minimalSpeed);
  Serial.print(" heading ");
  Serial.print(heading);
  Serial.print(" pitch ");
  Serial.print(pitch);
  Serial.print(" modeCalibHeading ");
  Serial.print(modeCalibHeading);
  Serial.print (" wantedHeading : ");
  Serial.print (wantedHeading);
  Serial.print (" wantedpotentiometer : ");
  Serial.print (wantedpotentiometer);
  Serial.print (" filteredPotError : ");
  Serial.print (filteredPotError);
  Serial.print (" cmd : ");
  Serial.print (cmd);
  Serial.print (" pHeading : ");
  Serial.print (KpHeading * pHeading);
  Serial.print (" dHeading : ");
  Serial.print (KdHeading*filteredDHeading);
  Serial.print (" iHeading : ");
  Serial.print (KiHeading*iHeading);
  Serial.println(); 
}

/*
      if (millis()-delayTime>200 and abs(actualSpeed)>minimalSpeed and Potentiometer == lastPotentiometer) {
      minimalSpeed = abs(actualSpeed);
      delayTime = millis();
      lastPotentiometer = Potentiometer;
    }
    else if (millis()-delayTime>200) {
      delayTime = millis();
      lastPotentiometer = Potentiometer;
    }
*/

