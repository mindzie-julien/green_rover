#!/usr/bin/env python3

import smbus2 as smbus

import time

import RPi.GPIO as GPIO



def send_string_to_arduino(string):

    # Convertir la chaîne en tableau de caractères

    data = list(string)

    print(data)

    # Envoyer chaque caractère à l'Arduino

    for char in data:

        bus.write_byte(addr, ord(char))

        print(ord(char))

        time.sleep(1)  # Attendez un peu entre chaque envoi pour que l'Arduino puisse traiter



addr = 0x9 #bus address

bus = smbus.SMBus(1)

time.sleep(1)



def contact_callback(channel):

    if (GPIO.input(channel) == 0 or GPIO.input(channel) == 1):

        print("Détection Jack")



        message = ("E")

        send_string_to_arduino(message)

        print("Message envoyé à l'Arduino avec succès.")

        time.sleep(0.1)



        message = ("wb")

        send_string_to_arduino(message)

        print("Message envoyé à l'Arduino avec succès.")

        time.sleep(2)



        message = ("as")

        send_string_to_arduino(message)

        print("mes")

        time.sleep(0.5)

        

        message = ("E")

        send_string_to_arduino(message)

        print("mess")

        time.sleep(0.1)



        message = ("ap")

        send_string_to_arduino(message)

        print("Message envoyé à l'Arduino avec succès.")

        time.sleep(0.5)

        

        message = ("E")

        send_string_to_arduino(message)

        print("Message envoyé à l'Arduino avec succès.")

        time.sleep(0.1)

        

        message = ("xa")

        send_string_to_arduino(message)

        print("Message envoyé à l'Arduino avec succès.")

        time.sleep(6)



        message = ("at")

        send_string_to_arduino(message)

        print("message")

        time.sleep(1)

        

        message = ("E")

        send_string_to_arduino(message)

        print("Message envoyé à l'Arduino avec succès.")

        time.sleep(0.5)

        

        for i in range(3):

            message = ("na")

            send_string_to_arduino(message)

            print("Message envoyé à l'Arduino avec succès.")

            time.sleep(1)

            message = ("au")

            send_string_to_arduino(message)

            print("Message envoyé à l'Arduino avec succès.")

            time.sleep(1)

            message = ("E")

            send_string_to_arduino(message)

            print("Message envoyé à l'Arduino avec succès.")

            time.sleep(0.1)



        message = ("as")

        send_string_to_arduino(message)

        print("mes")

        time.sleep(0.5)



        message = ("E")

        send_string_to_arduino(message)

        print("mess")

        time.sleep(0.1)



        message = ("k")

        send_string_to_arduino(message)

        print("Message")

        time.sleep(0.1)



        message = ("E")

        send_string_to_arduino(message)

        print(message)

        time.sleep(0.1)



        message = ("sa")

        send_string_to_arduino(message)

        print(message)

        time.sleep(1)



        message = ("E")

        send_string_to_arduino(message)

        print("Message")

        time.sleep(100000)



GPIO.setmode(GPIO.BCM)

GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.add_event_detect(5, GPIO.BOTH, callback=contact_callback, bouncetime=500)

input("Appuyez sur Entrée pour quitter...\n")

GPIO.cleanup()

