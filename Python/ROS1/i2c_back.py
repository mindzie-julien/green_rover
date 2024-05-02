#!/usr/bin/env python3

import rospy

import smbus2 as smbus

from smbus2 import i2c_msg

import time



def receive_data():

    try:

        # Lecture de données depuis l'esclave

        

        

        # Lecture d'un octet de l'esclave

        #length = bus.read_byte(addr)

        

        # Ajout de l'octet à la chaîne de données reçue

        #for _ in range(2):

            #received_data += chr(bus.read_i2c_block_data(addr, 0, 2))

        received_data = bus.read_byte(addr)

        

        return received_data

    

    except KeyboardInterrupt:

        # Arrêt du programme si Ctrl+C est pressé

        print("Arrêt du programme")



if __name__ == '__main__':

    rospy.init_node("i2c_back")

    rospy.loginfo("Test node i2c moteur has been started.")

    

    

    addr = 0x9 #bus address

    bus = smbus.SMBus(1)

    time.sleep(1)

    

    while not rospy.is_shutdown():

        try:

            received_data = receive_data()

            if received_data:

                print("Chaine : ", received_data)

            time.sleep(0.5)

        except Exception as e:

           print("Erreur : ", str(e))

