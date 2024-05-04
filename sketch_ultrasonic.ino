#include "NewPing.h"

#define TRIGGER_PIN_1  7
#define ECHO_PIN_1     7
#define TRIGGER_PIN_2  5
#define ECHO_PIN_2     5
#define TRIGGER_PIN_3  11
#define ECHO_PIN_3     11
#define TRIGGER_PIN_4  10
#define ECHO_PIN_4     10
#define TRIGGER_PIN_5  9
#define ECHO_PIN_5     9
#define TRIGGER_PIN_6  8
#define ECHO_PIN_6     8
#define MAX_DISTANCE 400

NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE); //ag
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE); //a
NewPing sonar3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE); //ad
NewPing sonar4(TRIGGER_PIN_4, ECHO_PIN_4, MAX_DISTANCE); //dg
NewPing sonar5(TRIGGER_PIN_5, ECHO_PIN_5, MAX_DISTANCE); //d
NewPing sonar6(TRIGGER_PIN_6, ECHO_PIN_6, MAX_DISTANCE); //dd

// Define Variables

float distance1; // Stores calculated distance in cm for First Sensor
float distance2; // Stores calculated distance in cm for Second Sensor
float distance3;
float distance4; // Stores calculated distance in cm for First Sensor
float distance5; // Stores calculated distance in cm for Second Sensor
float distance6;
float seuil = 20;
//json et I2C
int u1 ;
int u2 ;
int u3 ;
int u4 ;
int u5 ;
int u6 ;
/********************************************/
void setup() {
  Serial.begin (9600);
}


void loop()
{
  // Measure duration for first sensor    
  distance1 = sonar1.ping_cm(MAX_DISTANCE);
  if (distance1 != 0 && distance1 < seuil){
    u1 = 1;  
  }
  else{
    u1 = 0; // obstacle devant_gauche
  }
  distance2 = sonar2.ping_cm(MAX_DISTANCE);
  if (distance2 != 0 && distance2 < seuil){
    u2 = 1;  
  }
  else{
    u2 = 0; // obstacle devant_gauche
  }
    distance3 = sonar3.ping_cm(MAX_DISTANCE);
  if (distance3 != 0 && distance3 < seuil){
    u3 = 1;  
  }
  else{
    u3 = 0; // obstacle devant_gauche
  }
    distance4 = sonar4.ping_cm(MAX_DISTANCE);
  if (distance4 != 0 && distance4 < seuil){
    u4 = 1;  
  }
  else{
    u4 = 0; // obstacle devant_gauche
  }
    distance5 = sonar5.ping_cm(MAX_DISTANCE);
  if (distance5 != 0 && distance5 < seuil){
    u5 = 1;  
  }
  else{
    u5 = 0; // obstacle devant_gauche
  }
    distance6= sonar6.ping_cm(MAX_DISTANCE);
  if (distance6 != 0 && distance6 < seuil){
    u6 = 1;  
  }
  else{
    u6 = 0; // obstacle devant_gauche
  }
}