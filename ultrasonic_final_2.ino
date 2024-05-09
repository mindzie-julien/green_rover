#include "NewPing.h"

#define TRIGGER_PIN_4  7
#define ECHO_PIN_4     7
#define TRIGGER_PIN_5  9
#define ECHO_PIN_5     9
#define TRIGGER_PIN_6  11
#define ECHO_PIN_6     11
#define TRIGGER_PIN_1  10
#define ECHO_PIN_1     10
#define TRIGGER_PIN_2  8
#define ECHO_PIN_2     8
#define TRIGGER_PIN_3  5
#define ECHO_PIN_3     5
#define MAX_DISTANCE 400
#define FORWARDPIN 13
#define BACKWARDPIN 12
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
float distance6;/*
float seuil_DAv = 50;
float seuil_GAv = 50;
float seuil_MAv = 50;
float seuil_DAr = 50;
float seuil_GAr = 50;
float seuil_MAr = 50;*/
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
  pinMode(FORWARDPIN, OUTPUT);
  pinMode(BACKWARDPIN, OUTPUT);
  digitalWrite(FORWARDPIN, LOW);
  digitalWrite(BACKWARDPIN, LOW);
}


void loop()
{ //sonar5.ping_cm(MAX_DISTANCE);
  // Measure duration for first sensor    
  delay(50);
  distance1 = sonar1.ping_cm(MAX_DISTANCE);
  if (distance1 != 0 && distance1 < 20){
    u1 = 1;
    digitalWrite(FORWARDPIN, HIGH);
    Serial.println("sonar 1 HIGH"); 
  }
  else{
    u1 = 0; // obstacle 
    //digitalWrite(FORWARDPIN, LOW);
    Serial.println("sonar 1 LOW");
    delay(50);
    distance2 = sonar2.ping_cm(MAX_DISTANCE);
    if (distance2 != 0 && distance2 < 35){
      u2 = 1;  
      digitalWrite(FORWARDPIN, HIGH);
          Serial.println("sonar 2 HIGH"); 

    }
    else{
      u2 = 0; // obstacle 
      //digitalWrite(FORWARDPIN, LOW);
      delay(50);
      distance3 = sonar3.ping_cm(MAX_DISTANCE);
      if (distance3 != 0 && distance3 < 20){
        u3 = 1;  
        digitalWrite(FORWARDPIN, HIGH);
            Serial.println("sonar 3 HIGH"); 

      }
      else{
        u3 = 0; // obstacle 
        digitalWrite(FORWARDPIN, LOW);
      }
    }
  } 
   delay(50);
   distance4 = sonar4.ping_cm(MAX_DISTANCE);
  if (distance4 != 0 && distance4 < 15){
    u4 = 1;
    digitalWrite(BACKWARDPIN, HIGH);
    Serial.println("sonar 4 HIGH"); 
  }
  else{
    u4 = 0; // obstacle 
    //digitalWrite(BACKWARDPIN, LOW);
    Serial.println("sonar 4 LOW");
    delay(50);
    distance5 = sonar5.ping_cm(MAX_DISTANCE);
    if (distance5 != 0 && distance5 < 15){
      u5 = 1;  
      digitalWrite(BACKWARDPIN, HIGH);
          Serial.println("sonar 5 HIGH"); 

    }
    else{
      u5 = 0; // obstacle 
      //digitalWrite(BACKWARDPIN, LOW);
      delay(50);
      distance6 = sonar6.ping_cm(MAX_DISTANCE);
      if (distance6 != 0 && distance6 < 15){
        u6 = 1;  
        digitalWrite(BACKWARDPIN, HIGH);
            Serial.println("sonar 6 HIGH"); 

      }
      else{
        u6 = 0; // obstacle 
        digitalWrite(BACKWARDPIN, LOW);
      }
    }
  }
}
