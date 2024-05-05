import RPi.GPIO as GPIO
import pigpio
import time
servo_L = 17     #Use pin 11 for PWM signal BCM17
servo_R = 18  #Use pin 12 for PWM signal BCM18
#intialisation of  pigpio
pwm_L = pigpio.pi()
pwm_R = pigpio.pi()
#config of signal pwm
pwm_L.set_mode(servo_L, pigpio.OUTPUT)
pwm_L.set_PWM_frequency(servo_L, 50)

pwm_R.set_mode(servo_R, pigpio.OUTPUT)
pwm_R.set_PWM_frequency(servo_R, 50)

def pince_position(position):
    if position == "o":
        for i in range(0, 501, 1):
            pwm_L.set_servo_pulsewidth(servo_L, i+500) #500--1000
            pwm_R.set_servo_pulsewidth(servo_R, int(2500-7/5*i)) #2500--1800
            time.sleep(0.005)
    elif position == "f":
        for i in range(500, -1, -1):
            pwm_L.set_servo_pulsewidth(servo_L, i+500) #500--1000
            pwm_R.set_servo_pulsewidth(servo_R, int(2500-7/5*i)) #2500-->
            time.sleep(0.005)
while True:
    p=input("position:")
    pince_position(p)
