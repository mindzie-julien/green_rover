#include <Wire.h>

#define SLAVE_ADDRESS 9

void setup() {
  // Démarre la communication série
  Serial.begin(9600);
  
  // Initialise la communication I2C en tant que maître
  Wire.begin();
}

void loop() {
  // Envoie un nombre sur le port 9 via I2C
  char nombre = "e"; // Changer ici pour le nombre que vous souhaitez envoyer
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(nombre);
  Wire.endTransmission();
  
  // Attente pour la prochaine itération
  delay(1000); // Attente d'une seconde avant d'envoyer à nouveau
}
