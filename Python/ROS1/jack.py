#!/usr/bin/env python3

import rospy

import time

import RPi.GPIO as GPIO





if __name__ == '__main__':

    rospy.init_node("jack")

    rospy.loginfo("Test jack")

    

    while not rospy.is_shutdown():

        pinBtn = 4



        GPIO.setmode(GPIO.BOARD)



        GPIO.setup(pinBtn, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)



        etat = GPIO.input(pinBtn)

        print(etat)

        time.sleep(1)

