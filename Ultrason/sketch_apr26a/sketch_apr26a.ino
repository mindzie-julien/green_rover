// defines pins numbers 

#include <Wire.h>
const int trigPin = 5; 

const int echoPin = 4; // defines variables 

long duration; 

int distance; 
int detect = 0;
void setup() 

{

 pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output 

pinMode(echoPin, INPUT); // Sets the echoPin as an Input 
Wire.begin(9);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
Serial.begin(19200); // Starts the serial communication

 } 

void loop() 

{ 

// Clears the trigPin 

digitalWrite(trigPin, LOW); 

delayMicroseconds(2); // Sets the trigPin on HIGH state for 10 micro seconds 

digitalWrite(trigPin, HIGH); 

delayMicroseconds(10); 

digitalWrite(trigPin, LOW); // Reads the echoPin, returns the sound wave travel time in microseconds 

duration = pulseIn(echoPin, HIGH); // Calculating the distance 

distance = duration * 0.034 / 2; // Prints the distance on the Serial Monitor 

Serial.print("Distance: "); 

Serial.println(distance);

if (distance <= 15){
  detect = 2;
}else{
  detect = 1;
}
Serial.print("Détection: "); 

Serial.println(detect);
requestEvent(); 

}

void requestEvent() {
  Wire.write(detect); // respond with message of 6 bytes
  // as expected by master
}
