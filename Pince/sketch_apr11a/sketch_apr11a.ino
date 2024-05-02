#include <Servo.h>
#include <Wire.h>

#define I2C_SLAVE_ADDRESS 8  // Adresse I2C de l'Arduino

Servo monServo;  // Création d'un objet Servo
Servo monServo1;  // Création d'un objet Servo
Servo monServo2;  // Création d'un objet Servo

int angle;
int angle1;
int angle2;

void setup() {
  monServo.attach(9);  // Attacher le servomoteur à la broche 9
  monServo1.attach(10);  // Attacher le servomoteur à la broche 9
  monServo2.attach(11);  // Attacher le servomoteur à la broche 9
  Wire.begin(I2C_SLAVE_ADDRESS);                                               // Initialisation de la communication I2C
  Wire.onReceive(receiveEvent);                                                // Définition de la fonction à appeler lors de la réception de données
  Serial.begin(9600);                                                          // Initialisation de la communication série pour le débogage
  Serial.println("-----------------------------------> code initialization");  // Message de démarrage
}

void loop() {
  delay(100);  // Attendre 1 seconde
}

void receiveEvent(int numBytes) {
  while (Wire.available()) {
    int c = Wire.read();
    int pulseWidth = map(angle, 0, 180, 1000, 2000);
    int pulseWidth1 = map(angle1, 0, 180, 544, 2400);
    int pulseWidth2 = map(angle2, 0, 180, 544, 2400);
    
    if (c == 0){
      Serial.println("-----> OFF");
      angle = 60;
      angle1 = 50;
      angle2 = 200;
      pulseWidth = map(angle, 0, 180, 1000, 2000);
      monServo.writeMicroseconds(pulseWidth);
      pulseWidth1 = map(angle1, 0, 180, 544, 2400);
      monServo1.writeMicroseconds(pulseWidth1);
      pulseWidth2 = map(angle2, 0, 180, 544, 2400);
      monServo2.writeMicroseconds(pulseWidth2);
    }
    
    else if (c == 1){
      Serial.println("-----> Haut pince");
      angle = 200;
      pulseWidth = map(angle, 0, 180, 1000, 2000);
      monServo.writeMicroseconds(pulseWidth);
    }

    else if (c == 2){
      Serial.println("-----> Bas pince");
      angle = 60;
      pulseWidth = map(angle, 0, 180, 1000, 2000);
      monServo.writeMicroseconds(pulseWidth);
    }

    else if (c == 3){
      Serial.println("-----> Pince droite ouverte");
      angle1 = 200;
      pulseWidth1 = map(angle1, 0, 180, 544, 2400);
      monServo1.writeMicroseconds(pulseWidth1);
    }

    else if (c == 4){
      Serial.println("-----> Pince droite fermé");
      angle1 = 50;
      pulseWidth1 = map(angle1, 0, 180, 544, 2400);
      monServo1.writeMicroseconds(pulseWidth1);
    }

    else if (c == 5){
      Serial.println("-----> Pince gauche ouverte");
      angle2 = 40;
      pulseWidth2 = map(angle2, 0, 180, 544, 2400);
      monServo2.writeMicroseconds(pulseWidth2);
    }
      
    else if (c == 6){
      Serial.println("-----> Pince gauche fermé");
      angle2 = 200;
      pulseWidth2 = map(angle2, 0, 180, 544, 2400);
      monServo2.writeMicroseconds(pulseWidth2);
    }

  }
}
