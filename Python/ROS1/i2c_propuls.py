#!/usr/bin/env python3

import rospy

import smbus2 as smbus

import time



def send_string_to_arduino(string):

    # Convertir la chaîne en tableau de caractères

    data = list(string)

    

    # Envoyer chaque caractère à l'Arduino

    for char in data:

        bus.write_byte(addr, ord(char))

        time.sleep(1)  # Attendez un peu entre chaque envoi pour que l'Arduino puisse traiter



if __name__ == '__main__':

    rospy.init_node("i2c_moteur")

    rospy.loginfo("Test node i2c moteur has been started.")

    

    

    addr = 0x9 #bus address

    bus = smbus.SMBus(1)

    time.sleep(1)

    

    while not rospy.is_shutdown():    

        try:

           # Exemple d'envoi d'une chaîne de caractères à l'Arduino

           message = input("Instruction : ")

           send_string_to_arduino(message)

           print("Message envoyé à l'Arduino avec succès.")

    

        except Exception as e:

           print("Erreur lors de l'envoi du message à l'Arduino:", str(e))

