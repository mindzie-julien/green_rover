#!/usr/bin/env python3
#-- coding: utf-8 --
import RPi.GPIO as GPIO
import time


#Set function to calculate percent from angle
def angle_to_percent (angle) :
    if angle > 180 or angle < 0 :
        return False

    start = 2.5
    end = 12.5
    ratio = (end - start)/180 #Calcul ratio from angle to percent
    angle_as_percent = angle * ratio

    return start + angle_as_percent


GPIO.setmode(GPIO.BOARD) #Use Board numerotation mode
GPIO.setwarnings(False) #Disable warnings


servo_left = 11     #Use pin 11 for PWM signal
servo_right = 12    #Use pin 12 for PWM signal
frequence = 50
GPIO.setup(servo_left, GPIO.OUT)
GPIO.setup(servo_right, GPIO.OUT)
pwm_left = GPIO.PWM(servo_left, frequence)
pwm_right = GPIO.PWM(servo_right, frequence)

def open_left_clamp():
    #Initial position
    pwm_left.start(angle_to_percent(90))

def close_left_clamp(step=1):
    # augmenter vitesse --> step > 1
    # dimunier vitesse  --> 0 < step < 1
    duty = 1
    while (duty < 80):
        pwm_left.ChangeDutyCycle(angle_to_percent(duty))
        time.sleep(0.01)
        duty += step

def open_right_clamp():
    #Initial position
    pwm_right.start(angle_to_percent(60))

def close_right_clamp(step=1):
    # augmenter vitesse --> step > 1
    # dimunier vitesse  --> 0 < step < 1
    duty = 159
    while (duty > 70):
        pwm_right.ChangeDutyCycle(angle_to_percent(duty))
        time.sleep(0.01)
        duty -= step


open_right_clamp()
time.sleep(0.3)
open_left_clamp()
time.sleep(0.3)
close_right_clamp(0.75)
time.sleep(0.3)
close_left_clamp(0.75)



#Close GPIO & cleanup
pwm_left.stop()
pwm_right.stop()
GPIO.cleanup()
