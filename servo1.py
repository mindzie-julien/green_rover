import RPi.GPIO as GPIO
import pigpio
import time
servo_L = 17     #Use pin 11 for PWM signal BCM17
servo_R = 18    #Use pin 12 for PWM signal BCM18

#initialisation de pigpio
pwm_L = pigpio.pi()
pwm_R = pigpio.pi()

pwm_L.set_mode(servo_L, pigpio.OUTPUT)
pwm_L.set_PWM_frequency(servo_L, 10000)

pwm_R.set_mode(servo_R, pigpio.OUTPUT)
pwm_R.set_PWM_frequency(servo_R, 50)

#def open_pince 

while True:
    """
    for i in range(2500, 1500, -1):
        pwm_L.set_servo_pulsewidth(servo_L, i)
        print(i)
        time.sleep(0.001)



    for i in range(1500, 2501, 1):
        pwm_L.set_servo_pulsewidth(servo_L, i)
        print(i)
        time.sleep(0.001)
    """
    c=input()
    if c=="q":
        pwm_L.set_PWM_dutycycle(servo_L, 0)
        pwm_R.set_PWM_dutycycle(servo_R, 0)
        pwm_L.set_PWM_frequency(servo_L, 0)
        pwm_R.set_PWM_frequency(servo_R, 0)

    elif c=="f":
        pwm_L.set_servo_pulsewidth(servo_L, 500)
        pwm_R.set_servo_pulsewidth(servo_R, 2500)

    elif c == "o":
        pwm_L.set_servo_pulsewidth(servo_L, 1000)
        pwm_R.set_servo_pulsewidth(servo_R, 1800)

