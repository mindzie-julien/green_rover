#!/usr/bin/env python3

import rospy

import smbus2 as smbus

import time



def send_string_to_arduino(string):

    encoded_data = string.encode()

    bus.write_i2c_block_data(addr, 0, list(encoded_data))



def receive_data():

    # Lecture de données depuis l'esclave

    

        

    # Ajout de l'octet à la chaîne de données reçue

    time.sleep(0.1)

    received_data = bus.read_byte(addr)

        

    

    return received_data



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

           if (message == "E"):

               message_recu = False

               while not message_recu:

                   received_data = receive_data()

                   if received_data:

                       print("Données : ", received_data)

                       message_recu = True         

    

        except Exception as e:

           print("Erreur lors de l'envoi du message à l'Arduino:", str(e))

