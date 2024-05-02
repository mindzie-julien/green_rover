#!/usr/bin/env python3



import RPi.GPIO as GPIO



def main():

    def contact_callback(channel):

        if GPIO.input(channel):

            print("Détection Jack")       



    GPIO.setmode(GPIO.BCM)

    GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    GPIO.add_event_detect(4, GPIO.BOTH, callback=contact_callback, bouncetime=300)

    input("Appuyez sur Entrée pour quitter...\n")

    GPIO.cleanup()



if __name__ == '__main__':

    main()

