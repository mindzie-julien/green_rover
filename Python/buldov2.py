#!/usr/bin/env python3

import smbus2 as smbus

import time

import RPi.GPIO as GPIO



def receive_data():

    addre = 0x9 #bus address

    bus = smbus.SMBus(1)

    time.sleep(0.2)

    try:

        received_data = bus.read_byte(addre)

        return received_data

    except KeyboardInterrupt:

        # Arrêt du programme si Ctrl+C est pressé

        print("Arrêt du programme")



def receive_data_big():

    addres = 0x8 #bus address

    bus = smbus.SMBus(1)

    time.sleep(0.2)

    try:

        received_data_big = bus.read_byte(addres)

        return received_data_big

    except KeyboardInterrupt:

        # Arrêt du programme si Ctrl+C est pressé

        print("Arrêt du programme")



def send_string_to_arduino(string):

    addr = 0x8 #bus address

    bus = smbus.SMBus(1)

    time.sleep(0.1)

    encoded_data = string.encode()

    bus.write_i2c_block_data(addr, 0, list(encoded_data))



def contact_callback(channel):

    received_data_big = 0

    if (GPIO.input(channel) == 0 or GPIO.input(channel) == 1):

        print("Détection Jack")



        message = ("E")

        send_string_to_arduino(message)

        print("Message envoyé à l'Arduino avec succès.")

        time.sleep(0.1)



        message = ("r100a")

        send_string_to_arduino(message)

        print("Message envoyé à l'Arduino avec succès.")

        for i in range(10):

            try:

                received_data = receive_data()

                if received_data:

                    print("Chaine : ", received_data)

                    if (received_data == 2):

                        message = ("E")

                        send_string_to_arduino(message)

                        time.sleep(0.5)

                        received_data_big = receive_data_big()

                        received_data = receive_data()

                        while (received_data == 2):

                            message = ("E")

                            send_string_to_arduino(message)

                            time.sleep(0.5)

                            received_data = receive_data()

                        if (received_data_big > 0):

                            if (received_data_big >= 100):

                                consigne = 'r' + str(received_data_big) + 'a'

                                print(consigne)

                                message = consigne

                                send_string_to_arduino(message)

                                received_data_big = 0

                            elif (received_data_big < 100):

                                consigne = 'r' + '0' + str(received_data_big) + 'a'

                                print(consigne)

                                message = consigne

                                send_string_to_arduino(message)

                                received_data_big = 0

                time.sleep(0.5)

            except Exception as e:

                print("Erreur : ", str(e))



        message = ("E")

        send_string_to_arduino(message)

        print("Message")

        time.sleep(100000)



GPIO.setmode(GPIO.BCM)

GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.add_event_detect(5, GPIO.BOTH, callback=contact_callback, bouncetime=500)

input("Appuyez sur Entrée pour quitter...\n")

GPIO.cleanup()

