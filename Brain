// For MPU
#include "MPUDefine.h"
#include <Wire.h>

typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;    uint8_t x_accel_l;
    uint8_t y_accel_h;    uint8_t y_accel_l;
    uint8_t z_accel_h;    uint8_t z_accel_l;
    uint8_t t_h;    uint8_t t_l;
    uint8_t x_gyro_h;    uint8_t x_gyro_l;
    uint8_t y_gyro_h;    uint8_t y_gyro_l;
    uint8_t z_gyro_h;    uint8_t z_gyro_l;
  } reg;
  struct 
  {
    int16_t x_accel;    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;    int16_t y_gyro;
    int16_t z_gyro;
  } value;
};
//For Sim900
int8_t answer;int x;int SMSPin= 2;char SMS[200]; 
int8_t sendATcommand(char* ATcommand, char* expected_answer, unsigned int timeout){
  
    uint8_t x=0,  answer=0;
    char response[100];
    unsigned long previous;
    memset(response, '\0', 100);    
    delay(100);
    while( Serial.available() > 0) Serial.read(); 
    Serial.println(ATcommand); 
        x = 0;
    previous = millis();
    do{  
        if(Serial.available() != 0){   
            response[x] = Serial.read();
            x++;
            if (strstr(response, expected_answer) != NULL)   
            {answer = 1;}}}
    while((answer == 0) && ((millis() - previous) < timeout));   
    return answer;
}

//Some thing called boolean *sexy*
volatile boolean ShakeAlert = false;
volatile boolean MQ3Alert = false;
volatile boolean BPMAlert1= false;
volatile boolean Run      = false;
volatile boolean Walk     = false;
volatile boolean Hot      = false;
volatile boolean Cold     = false;
volatile boolean Sick     = false;


//Some thing called int *meow*
int MQ3 =       A1;
int Shake =     A2;  

volatile int  ShakeAnaC = 0;
volatile int  MQ3AnaC = 0;
volatile int  BPMAnaC = 0;
//  VARIABLES
int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 13;                // pin to blink led at each beat
int fadePin = 5;                  // pin to do fancy classy fading blink at each beat
int fadeRate = 0;                 // used to fade LED on with PWM on fadePin

// these variables are volatile because they are used during the interrupt service routine!
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, must be seeded! 
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.
volatile int timeA = millis();
volatile int timeB;
//display
#include "Adafruit_ssd1306syp.h"
#define SDA_PIN 8
#define SCL_PIN 9
Adafruit_ssd1306syp display(SDA_PIN,SCL_PIN);


void setup(){ 
  //for OLED
  delay(1000);                      // need 1s to start the machine
  display.initialize();             // install the OLED first
  //for MPU
  uint8_t c; Wire.begin();
  pinMode(blinkPin,OUTPUT);         // pin that will blink to your heartbeat!
  pinMode(fadePin,OUTPUT);          // pin that will fade to your heartbeat!
  Serial.begin(115200);             // we agree to talk fast!
  interruptSetup();
  
  // sets up to read Pulse Sensor signal every 2mS 

}



void loop(){
  // 
  double dT;
  accel_t_gyro_union accel_t_gyro;
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);

  dT = ( (double) accel_t_gyro.value.temperature + 12412.0) / 340.0;
    if (dT >36) { 
      if (dT > 38) { 
        if (dT > 42) {Sick = false; Hot = true;} 
        else { Sick = true;  Hot= false;}}
      else {   Sick = false; Hot = false;}}
    else {     Sick = false; Hot = false; Cold = true;} 
  int i = 0; 
  volatile int D[10];volatile int x[10]; volatile int y[10]; volatile int z[10];

  if   (i < 10) {
    i ++ ;
  x[i] = abs(accel_t_gyro.value.x_accel);  y[i] = abs(accel_t_gyro.value.y_accel);  z[i] = abs(accel_t_gyro.value.z_accel);
  if (i > 0) {
  D[i] = (x[i]*x[i-1]+ y[i]*y[i-1] + z[i]*z[i-1])
/(sqrt(x[i] * x[i] + y[i] * y[i] + z[i] * z[i]) * sqrt(x[i-1] * x[i-1] + y[i-1] * y[i-1] + z[i-1] * z[i-1]));}
  if (i = 10) {
  volatile int Move = ( D[0] + 2*D[1] + 3*D[2] + 4*D[3] + 5*D[4] + 6*D[5] + 7*D[6] + 8*D[7] + 9*D[8] + 10*D[9]) / 55;
  delay(100); i = 0; }}
    
  //
  MQ3AlertVoid();
  ShakeAlertVoid();
  //
  timeB = millis() - timeA;
  if ( timeB >= 901000) {
      timeB = timeA;}
  display.update();
  display.clear();                        // clear the sceen
  display.setTextSize(1); 
  display.setTextColor(WHITE);
  volatile int timeA = millis();
  volatile int timeB = millis();

  sendDataToProcessing('S', Signal);     // send Processing the raw Pulse Sensor data
  if (QS == true){                       // Quantified Self flag is true when arduino finds a heartbeat
        fadeRate = 255;                  // Set 'fadeRate' Variable to 255 to fade LED with pulse
        sendDataToProcessing('B',BPM);   // send heart rate with a 'B' prefix
        sendDataToProcessing('Q',IBI);   // send time between beats with a 'Q' prefix
        display.setCursor(3,3); display.print("BPM: "); display.println(BPM);     //print BPM to the OLED
        display.setCursor(4,4); display.print("IBI: "); display.println(IBI);     //print IBI to the OLED
        QS = false;                      // reset the Quantified Self flag for next time    
     }
  
  ledFadeToBeat();

  display.update();
  display.clear();                        // clear the sceen
  delay(20);   //  take a break
  
}


void ledFadeToBeat(){
    fadeRate -= 15;                         //  set LED fade value
    fadeRate = constrain(fadeRate,0,255);   //  keep LED fade value from going into negative numbers!
    analogWrite(fadePin,fadeRate);          //  fade LED
  }

void ShakeAlertVoid(){
    volatile int ShakeAnaA = analogRead(Shake);
    volatile int ShakeAnaB = abs(analogRead(Shake) - ShakeAnaA);
    volatile int ShakeAnaC;
    if (ShakeAnaB > 512) {  
      ShakeAnaC ++ ;}
    if (timeB == 900000) {
      if (ShakeAnaC > 7500) {
           ShakeAlert = true ;} 
      else {
           ShakeAlert = false;
           ShakeAnaC = 0;}}}
           
void MQ3AlertVoid(){ 
    volatile int MQ3AnaA = analogRead(MQ3);
    volatile int MQ3AnaB = abs(analogRead(Shake) - MQ3AnaA);
    volatile int MQ3AnaC;
    if (MQ3AnaB > 512) {  
      MQ3AnaC ++ ;}
    if (timeB >= 900000) {
      if (MQ3AnaC >22500) {
           MQ3Alert = true ;} 
      else {
           MQ3Alert = false;
           MQ3AnaC = 0;}}} 
 
           
void PSAlertVoid(){ 
     volatile int PSAnaC;
    if (BPM > 120) {  
      BPMAnaC ++ ;}
    if (timeB >= 900000) {
      if (BPMAnaC > 675) {
           BPMAlert1 = true;
      int k = 0; if (k = 0) {
        
 pinMode(3,OUTPUT);
 sendATcommand("AT","OK",2000);
 delay(3000);
 sendATcommand("ATD0908484900;","OK",2000); k ++;}
  
         } 
      else {
           BPMAlert1 = false;
           BPMAnaC = 0;}}}                    
      
void sendDataToProcessing(char symbol, int data ){
    Serial.print(symbol);                // symbol prefix tells Processing what type of data is coming
    Serial.println(data);                // the data to send culminating in a carriage return
  }
