#include "NewPing.h"
#include <ArduinoJson.h>
#include <Wire.h>

#define TRIGGER_PIN_1  7//13
#define ECHO_PIN_1     7//13
#define TRIGGER_PIN_2  5//A1//3//4//12
#define ECHO_PIN_2     5//A1//3//4//12
#define TRIGGER_PIN_3  11
#define ECHO_PIN_3     11
#define TRIGGER_PIN_4  10
#define ECHO_PIN_4     10
#define TRIGGER_PIN_5  9
#define ECHO_PIN_5     9
#define TRIGGER_PIN_6  8
#define ECHO_PIN_6     8
#define MAX_DISTANCE 400

int forwardPin = 13;
int backwardPin = 12;

NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE); //ag
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE); //a
NewPing sonar3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE); //ad
NewPing sonar4(TRIGGER_PIN_4, ECHO_PIN_4, MAX_DISTANCE); //dg
NewPing sonar5(TRIGGER_PIN_5, ECHO_PIN_5, MAX_DISTANCE); //d
NewPing sonar6(TRIGGER_PIN_6, ECHO_PIN_6, MAX_DISTANCE); //dd

// Define Variables
float duration1; // Stores First HC-SR04 pulse duration value
float duration2; // Stores Second HC-SR04 pulse duration value
float duration3; // Stores Second HC-SR04 pulse duration value
float duration4; // Stores First HC-SR04 pulse duration value
float duration5; // Stores Second HC-SR04 pulse duration value
float duration6; // Stores Second HC-SR04 pulse duration value

float distance1; // Stores calculated distance in cm for First Sensor
float distance2; // Stores calculated distance in cm for Second Sensor
float distance3;
float distance4; // Stores calculated distance in cm for First Sensor
float distance5; // Stores calculated distance in cm for Second Sensor
float distance6;
float soundcm;  // Stores calculated speed of sound in cm/ms
int iterations = 5;
float seuil = 35;
//json et I2C
int u1 ;
int u2 ;
int u3 ;
int u4 ;
int u5 ;
int u6 ;

String json;
StaticJsonDocument<300> doc;
const byte I2C_SLAVE_ADD = 0x8;
const byte I2C_LENGTH_LIMIT = 32;
const byte ASK_FOR_LENGTH = 0x0;
const byte ASK_FOR_DATA = 0x1;
char request = ' ';
byte requestIndex = 0;
/********************************************/
void setup() {
  //Serial.begin (9600);
  Serial.begin(9600);
  //serializeObject();
  Wire.begin(I2C_SLAVE_ADD);  
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  pinMode(forwardPin, OUTPUT);
  pinMode(backwardPin, OUTPUT);
  digitalWrite(forwardPin, LOW);
  digitalWrite(backwardPin, LOW);
  Serial.print("Json size: "); Serial.print(json.length()); Serial.println(" bytes");
  Serial.print("Json: "); Serial.print(json.c_str()); Serial.println("");
  Serial.print("Json: "); Serial.print(json.c_str()); Serial.println(""); 
  delay(1000); 
}
void receiveEvent(int bytes)
{
  while (Wire.available())
  {
    request = (char)Wire.read();
  }
}
void requestEvent() {
    doc["u1"] = u1; // obstacle devant_gauche
    doc["u2"] = u2; // obstacle devant
    doc["u3"] = u3;  // obstacle devant_droite
    doc["u4"] = u4; // obstacle derriere_gauche
    doc["u5"] = u5; // obstacle derriere
    doc["u6"] = u6;  // obstacle derriere_droite

    serializeJson(doc, json);
    if (request == ASK_FOR_LENGTH) {
        Wire.write(json.length());
        requestIndex = 0;
    }

    if (request == ASK_FOR_DATA) {
        if (requestIndex < (json.length() / I2C_LENGTH_LIMIT)) {
            Wire.write(json.c_str() + requestIndex * I2C_LENGTH_LIMIT, I2C_LENGTH_LIMIT);
            requestIndex++;
        }
        else {
            Wire.write(json.c_str() + requestIndex * I2C_LENGTH_LIMIT, json.length() % I2C_LENGTH_LIMIT);
            requestIndex = 0;
        }
    }
}

void loop()
{
  soundcm = 331.4 / 10000;

  // Measure duration for first sensor    
  duration1 = sonar1.ping_median(iterations);
  distance1 = (duration1 / 2) * soundcm;
  if (distance1 < seuil){
    u1 = 1;
    //digitalWrite(forwardPin, HIGH);
    //delay(100);
    Serial.println("sonar 1");
  }
  else{
    u1 = 0; // obstacle devant_gauche
    //digitalWrite(forwardPin, LOW);
    //delay(100);
    
  }
  // Add a delay between sensor readings
  //delay(50);
  // Measure duration for first sensor
  duration2 = sonar2.ping_median(iterations);
  distance2 = (duration2 / 2) * soundcm;
  if (distance2 < seuil){
    u2 = 1;
    Serial.println("near");
    digitalWrite(forwardPin, HIGH);
    //delay(100);
    Serial.println("sonar 2");
  }
  else{
    u2=0;
    Serial.println("far");
    digitalWrite(forwardPin, LOW);
    //delay(100);
  }
  //delay(70);
  // Measure duration for first sensor
  duration3 = sonar3.ping_median(iterations);
  distance3 = (duration3 / 2) * soundcm;
  if (distance3 < seuil){
    u3 = 1;
    //digitalWrite(forwardPin, HIGH);
    //delay(100);
    Serial.println("sonar 3");
  }
  else{
    u3=0;
    //digitalWrite(forwardPin, LOW);
    //delay(100);
  }
  //delay(70);
  // Calculate the distances for both sensors
  duration4 = sonar4.ping_median(iterations);
  distance4 = (duration4 / 2) * soundcm;
  if (distance4 < seuil){
    u4 = 1;
    //digitalWrite(backwardPin, HIGH);
    //delay(100);
    Serial.println("sonar 4");
  }
  else{
    u4=0;
    //digitalWrite(backwardPin, LOW );
    //delay(100);
  }
  // Add a delay between sensor readings
  //delay(70);
  // Measure duration for first sensor
  duration5 = sonar5.ping_median(iterations);
  distance5 = (duration5 / 2) * soundcm;
  if (distance5 < seuil){
    u5 = 1;
    //digitalWrite(backwardPin, HIGH);
    //delay(100);
    Serial.println("sonar 5");
  }
  else{
    u5=0;
    //digitalWrite(backwardPin, LOW);
    //delay(100);
  }
  //delay(70);
  // Measure duration for first sensor
  duration6 = sonar6.ping_median(iterations);
  distance6 = (duration6 / 2) * soundcm;
  if (distance6 < seuil){
    u6 = 1;
    //digitalWrite(backwardPin, HIGH);
    //delay(100);
    Serial.println("sonar 6");
  }
  else{
    u6=0;
    //digitalWrite(backwardPin, LOW);
    
  }
  //delay(70);
  /* Send results to Serial Monitor
    Serial.print("Distance 1: ");

    if (distance1 >= 400 || distance1 <= 2) {
    Serial.print("Out of range");
    }
    else {
    Serial.print(distance1);
    Serial.print(" cm ");

    }
    
    Serial.print("Distance 2: ");

    if (distance2 >= 400 || distance2 <= 2) {
    Serial.print("Out of range");
    }
    else {
    Serial.print(distance2);
    Serial.print(" cm");
    }
        Serial.print("Distance 3: ");

    if (distance3 >= 400 || distance3 <= 2) {
    Serial.print("Out of range");
    }
    else {
    Serial.print(distance3);
    Serial.print(" cm");
    }
  
  Serial.println(" ");
*/
}
