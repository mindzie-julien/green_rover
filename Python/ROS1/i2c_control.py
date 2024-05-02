#!/usr/bin/env python3

import rospy

import smbus2 as smbus

import time



if __name__ == '__main__':

    rospy.init_node("i2c_pince")

    rospy.loginfo("Test node i2c pince has been started.")

    

    while not rospy.is_shutdown():

        addr = 0x8 #bus address

        bus = smbus.SMBus(1)

        time.sleep(1)

        numb = 1

        while numb == 1:

            ledstate = input(">>>> ")

            if ledstate == "0":

                bus.write_byte(addr, 0x0)

            elif ledstate == "1":

                bus.write_byte(addr, 0x1)

            elif ledstate == "2":

                bus.write_byte(addr, 0x2)

            elif ledstate == "3":

                bus.write_byte(addr, 0x3)

            elif ledstate == "4":

                bus.write_byte(addr, 0x4)

            elif ledstate == "5":

                bus.write_byte(addr, 0x5)

            elif ledstate == "6":

                bus.write_byte(addr, 0x6)

            elif ledstate == "7":

                bus.write_byte(addr, 0x7)

    

            else:

                numb = 0

