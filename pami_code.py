#!/usr/bin/python3

import RPi.GPIO as GPIO
from time import sleep


# Definition des pins
M1_En = 21
M1_In1 = 20
M1_In2 = 16

M2_En = 18
M2_In1 = 23
M2_In2 = 24
jack = 5

# Creation d'une liste des pins pour chaque moteur pour compacter la suite du code
Pins = [[M1_En, M1_In1, M1_In2], [M2_En, M2_In1, M2_In2]]

def contact_callback (channel):
      if(GPIO.input(channel)== 1):
         print("Detection Jack")

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(M1_En, GPIO.OUT)
GPIO.setup(M1_In1, GPIO.OUT)
GPIO.setup(M1_In2, GPIO.OUT)

GPIO.setup(M2_En, GPIO.OUT)
GPIO.setup(M2_In1, GPIO.OUT)
GPIO.setup(M2_In2, GPIO.OUT)

GPIO.setmode(GPIO.BCM)
GPIO.setup(jack,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(jack, GPIO.BOTH, callback=contact_callback, bouncetime=500)
#input(True)
#GPIO.cleanup()

# Régler la vitesse des moteurs
M1_Vitesse = GPIO.PWM(M1_En, 100)
M2_Vitesse = GPIO.PWM(M2_En, 100)
M1_Vitesse.start(80) #Moteur Gauche
M2_Vitesse.start(70) #Moteur droit



def avancer(moteurNum) :
    GPIO.output(Pins[moteurNum - 1][1], GPIO.HIGH)
    GPIO.output(Pins[moteurNum - 1][2], GPIO.LOW)
    print("Moteur", moteurNum, "tourne en marche avant.")


def reculer(moteurNum) :
    GPIO.output(Pins[moteurNum - 1][1], GPIO.LOW)
    GPIO.output(Pins[moteurNum - 1][2], GPIO.HIGH)
    print("Moteur", moteurNum, "tourne en marche arrière.")

def arret(moteurNum) :
    GPIO.output(Pins[moteurNum - 1][1], GPIO.LOW)
    GPIO.output(Pins[moteurNum - 1][2], GPIO.LOW)
    print("Moteur", moteurNum, "arret.")

def arretComplet() :
    GPIO.output(Pins[0][1], GPIO.LOW)
    GPIO.output(Pins[0][2], GPIO.LOW)
    GPIO.output(Pins[1][1], GPIO.LOW)
    GPIO.output(Pins[1][2], GPIO.LOW)
    print("Moteurs arretes.")
arretComplet()

while True :
   # contact_callback(1)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(jack,GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(jack, GPIO.RISING) #callback=contact_callback, bouncetime=500)
    if GPIO.input(jack):
         print("Detection Jack")
         #sleep(90)
    # boucle principale
         reculer(1)  # Activer le sens 1 pour le moteur 1
         reculer(2)  # Activer le sens 1 pour le moteur 2
         sleep(2.98)  # Attendre pendant 2 secondes
         M1_Vitesse.start(45) #Moteur Gauche
         M2_Vitesse.start(41) #Moteur droit
         sleep(2.1)
         arretComplet()  # Arrêter les deux moteurs
         sleep(0.5)

         M1_Vitesse.start(80) #Moteur Gauche
         M2_Vitesse.start(70) #Moteur droit
    # Tourner de 90 degrés
         reculer(1)  # Activer le sens 1 pour le moteur 1
         avancer(2)  # Activer le sens 2 pour le moteur 2 (pour tourner)
         sleep(0.19)  # Attendre pendant 1.5 secondes pour tourner
         arretComplet()  # Arrêter les deux moteurs
         sleep(1)

    # Avancer pendant 3 secondes
         reculer(1)  # Activer le sens 1 pour le moteur 1
         reculer(2)  # Activer le sens 1 pour le moteur 2
         sleep(1)  # Attendre pendant 3 secondes
         M1_Vitesse.start(45) #Moteur Gauche
         M2_Vitesse.start(41) #Moteur droit
         sleep(2.1)
         arretComplet()  # Arrêter les deux moteurs

         break


