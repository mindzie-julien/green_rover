#!/usr/bin/python3

import RPi.GPIO as GPIO
import time

# Definition des pins pour le capteur ultrason
GPIO_TRIGGER = 26
GPIO_ECHO = 6

# Definition des pins pour les moteurs
M1_En = 21
M1_In1 = 20
M1_In2 = 16

M2_En = 18
M2_In1 = 23
M2_In2 = 24

# Creation d'une liste des pins pour chaque moteur pour compacter la suite du c>
Pins = [[M1_En, M1_In1, M1_In2], [M2_En, M2_In1, M2_In2]]

# Initialisation des GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Configuration des pins pour le capteur ultrason
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

# Configuration des pins pour les moteurs
for pin in Pins:
    GPIO.setup(pin, GPIO.OUT)

# Configuration des PWM pour régler la vitesse des moteurs
M1_Vitesse = GPIO.PWM(M1_En, 100)
M2_Vitesse = GPIO.PWM(M2_En, 100)
M1_Vitesse.start(100)
M2_Vitesse.start(100)

# Fonctions pour le mouvement des moteurs

def avancer(moteurNum, vitesse):
    GPIO.output(Pins[moteurNum - 1][1], GPIO.HIGH)
    GPIO.output(Pins[moteurNum - 1][2], GPIO.LOW)
    M1_Vitesse.ChangeDutyCycle(vitesse)
    M2_Vitesse.ChangeDutyCycle(vitesse)
    print("Moteur", moteurNum, "tourne en marche avant à la vitesse", vitesse)

def reculer(moteurNum, vitesse):
    GPIO.output(Pins[moteurNum - 1][1], GPIO.LOW)
    GPIO.output(Pins[moteurNum - 1][2], GPIO.HIGH)
    M1_Vitesse.ChangeDutyCycle(vitesse)
    M2_Vitesse.ChangeDutyCycle(vitesse)
    print("Moteur", moteurNum, "tourne en marche arrière à la vitesse", vitesse)

def arretComplet():
    GPIO.output(Pins[0][1], GPIO.LOW)
    GPIO.output(Pins[0][2], GPIO.LOW)
    GPIO.output(Pins[1][1], GPIO.LOW)
    GPIO.output(Pins[1][2], GPIO.LOW)
print("Moteurs arrêtés.")
    M1_Vitesse.ChangeDutyCycle(0)
    M2_Vitesse.ChangeDutyCycle(0)

# Fonction pour mesurer la distance avec le capteur ultrason
def distance():
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    TimeElapsed = StopTime - StartTime
distance = (TimeElapsed * 34300) / 2

    return distance
#time.sleep(91)
try:
    while True:

        dist = distance()
        print("Distance mesurée =", dist, "cm")
        if dist < 25:  # Si la distance est inférieure à 25 cm
            # Les moteurs fonctionnent à une vitesse réduite
            avancer(1,0)
            avancer(2,0)
        else:
            # Les moteurs fonctionnent à pleine vitesse

            avancer(1, 100)
            avancer(2, 100)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Arrêt du programme par l'utilisateur.")
    arretComplet()
    GPIO.cleanup()


