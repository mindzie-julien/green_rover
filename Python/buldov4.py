#!/usr/bin/env python3
import smbus2 as smbus
import pigpio
import time
import RPi.GPIO as GPIO

servo_L = 17     #Use pin 11 for PWM signal BCM17
servo_R = 18    #Use pin 12 for PWM signal BCM18

#initialisation de pigpio
pwm_L = pigpio.pi()
pwm_R = pigpio.pi()

pwm_L.set_mode(servo_L, pigpio.OUTPUT)
pwm_L.set_PWM_frequency(servo_L, 10000)

pwm_R.set_mode(servo_R, pigpio.OUTPUT)
pwm_R.set_PWM_frequency(servo_R, 50)


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

        pwm_L.set_servo_pulsewidth(servo_L, 1200)
        pwm_R.set_servo_pulsewidth(servo_R, 2500)
        time.sleep(0.1)
        pwm_L.set_servo_pulsewidth(servo_L, 1950)
        pwm_R.set_servo_pulsewidth(servo_R, 1300)
        time.sleep(2)
        pwm_L.set_servo_pulsewidth(servo_L, 1200)
        pwm_R.set_servo_pulsewidth(servo_R, 2500)

        message = ("E")
        send_string_to_arduino(message)
        print("Message envoyé à l'arduino avec succès.")
        time.sleep(0.1)
        message = ("E")
        send_string_to_arduino(message)
        time.sleep(0.5)

        message = ("m031a")
        send_string_to_arduino(message)
        print("Rotation")
        time.sleep(3)

        message = ("E")
        send_string_to_arduino(message)
        print("Message envoyé à l'Arduino avec succès.")
        time.sleep(0.1)
        message = ("E")
        send_string_to_arduino(message)
        time.sleep(0.5)

        message = ("r055b")
        send_string_to_arduino(message)
        print("Message envoyé à l'Arduino avec succès.")

        for i in range(4):
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
                                consigne = 'r' + str(received_data_big) + 'b'
                                print(consigne)
                                message = consigne
                                send_string_to_arduino(message)
                                received_data_big = 0
                            elif (received_data_big < 100):
                                consigne = 'r' + '0' + str(received_data_big) + 'b'
                                print(consigne)
                                message = consigne
                                send_string_to_arduino(message)
                                received_data_big = 0
                time.sleep(0.5)
            except Exception as e:
                print("Erreur : ", str(e))

        time.sleep(0.5)
        pwm_L.set_servo_pulsewidth(servo_L, 1800)
        pwm_R.set_servo_pulsewidth(servo_R, 1200)
        time.sleep(1)

        message = ("E")
        send_string_to_arduino(message)
        print("reset")
        time.sleep(0.5)

        message = ("r040a")
        send_string_to_arduino(message)
        print("Message envoyé à l'Arduino avec succès.")

        for i in range(4):
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

        time.sleep(0.5)
        pwm_L.set_servo_pulsewidth(servo_L, 1100)
        pwm_R.set_servo_pulsewidth(servo_R, 1800)
        time.sleep(0.5)
        time.sleep(0.5)
        pwm_L.set_servo_pulsewidth(servo_L, 1200)
        pwm_R.set_servo_pulsewidth(servo_R, 2500)
        time.sleep(1)

        message = ("E")
        send_string_to_arduino(message)
        print("Message envoyé à l'Arduino avec succès.")
        time.sleep(0.1)
        message = ("E")
        send_string_to_arduino(message)
        time.sleep(1)

        message = ("m180a")
        send_string_to_arduino(message)
        print("Rotation")
        time.sleep(3)

        message = ("E")
        send_string_to_arduino(message)
        time.sleep(0.1)
        message = ("E")
        send_string_to_arduino(message)
        print("Message envoyé à l'Arduino avec succès.")
        time.sleep(0.3)

        message = ("r075b")
        send_string_to_arduino(message)
        print("Message envoyé à l'Arduino avec succès.")

        for i in range(7):
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
                                consigne = 'r' + str(received_data_big) + 'b'
                                print(consigne)
                                message = consigne
                                send_string_to_arduino(message)
                                received_data_big = 0
                            elif (received_data_big < 100):
                                consigne = 'r' + '0' + str(received_data_big) + 'b'
                                print(consigne)
                                message = consigne
                                send_string_to_arduino(message)
                                received_data_big = 0
                time.sleep(0.5)
            except Exception as e:
                print("Erreur : ", str(e))

        time.sleep(0.5)
        pwm_L.set_servo_pulsewidth(servo_L, 2000)
        pwm_R.set_servo_pulsewidth(servo_R, 800)
        time.sleep(1)

        message = ("E")
        send_string_to_arduino(message)
        print("Message envoyé à l'Arduino avec succès.")
        time.sleep(0.1)

        message = ("k")
        send_string_to_arduino(message)
        print("Rotation")
        time.sleep(1)
        message = ("k")
        send_string_to_arduino(message)
        print("rotate")
        time.sleep(1)

        message = ("E")
        send_string_to_arduino(message)
        print("Message envoyé à l'Arduino avec succès.")
        time.sleep(0.5)
        message = ("E")
        send_string_to_arduino(message)
        time.sleep(0.1)

        message = ("r030b")
        send_string_to_arduino(message)
        print("Recule")
        time.sleep(2)

        message = ("E")
        send_string_to_arduino(message)
        print("Message envoyé à l'Arduino avec succès.")
        time.sleep(0.5)

        message = ("m165a")
        send_string_to_arduino(message)
        print("Rotation")
        time.sleep(4)

        message = ("E")
        send_string_to_arduino(message)
        print("ok")
        time.sleep(0.1)

        message = ("l")
        send_string_to_arduino(message)
        print("Rotation")
        time.sleep(1)
        message = ("l")
        send_string_to_arduino(message)
        print("Tu vas changer de sens ou merde")
        time.sleep(1)

        message = ("E")
        send_string_to_arduino(message)
        print("Message envoyé à l'Arduino avec succès.")
        time.sleep(0.1)
        message = ("E")
        send_string_to_arduino(message)
        time.sleep(0.1)

        message = ("r045b")
        send_string_to_arduino(message)
        print("Message envoyé à l'Arduino avec succès.")

        for i in range(6):
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
                                consigne = 'r' + str(received_data_big) + 'b'
                                print(consigne)
                                message = consigne
                                send_string_to_arduino(message)
                                received_data_big = 0
                            elif (received_data_big < 100):
                                consigne = 'r' + '0' + str(received_data_big) + 'b'
                                print(consigne)
                                message = consigne
                                send_string_to_arduino(message)
                                received_data_big = 0
                time.sleep(0.5)
            except Exception as e:
                print("Erreur : ", str(e))

        message = ("E")
        send_string_to_arduino(message)
        print("Message envoyé à l'Arduino avec succès.")
        time.sleep(0.5)

        message = ("m120a")
        send_string_to_arduino(message)
        print("Rotation")
        time.sleep(4)

        message = ("E")
        send_string_to_arduino(message)
        print("Message envoyé à l'Arduino avec succès.")
        time.sleep(0.1)
        time.sleep(0.5)
        pwm_L.set_servo_pulsewidth(servo_L, 1200)
        pwm_R.set_servo_pulsewidth(servo_R, 2500)
        time.sleep(1)
        message = ("E")
        send_string_to_arduino(message)
        time.sleep(0.1)

        message = ("r145b")
        send_string_to_arduino(message)
        print("Message envoyé à l'Arduino avec succès.")

        for i in range(6):
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
                                consigne = 'r' + str(received_data_big) + 'b'
                                print(consigne)
                                message = consigne
                                send_string_to_arduino(message)
                                received_data_big = 0
                            elif (received_data_big < 100):
                                consigne = 'r' + '0' + str(received_data_big) + 'b'
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
GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(16, GPIO.BOTH, callback=contact_callback, bouncetime=500)
input("Appuyez sur Entrée pour quitter...\n")
GPIO.cleanup()