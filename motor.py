
import RPi.GPIO as GPIO 
from time import sleep

GPIO.setmode(GPIO.BCM)
#Pins 18 22 24 GPIO 24 25 8
Motor1E = 24 #  Enable pin 1 of the controller IC
Motor1A = 25 #  Input 1 of the controller IC
Motor1B = 8 #  Input 2 of the controller IC


GPIO.setup(Motor1A,GPIO.OUT)
GPIO.setup(Motor1B,GPIO.OUT)
GPIO.setup(Motor1E,GPIO.OUT)

forward=GPIO.PWM(Motor1A,100) # configuring Enable pin for PWM
reverse=GPIO.PWM(Motor1B,100) # configuring Enable pin for PWM


if __name__ == '__main__' :
    forward.start(0) 
    reverse.start(0)

    # this will run your motor in reverse direction for 2 seconds with 80% speed by supplying 80% of the battery voltage
    print ("GO backward")
    GPIO.output(Motor1E,GPIO.HIGH)
    forward.ChangeDutyCycle(0)
    reverse.ChangeDutyCycle(100)
    sleep(2)


    # this will run your motor in forward direction for 5 seconds with 50% speed.
    print ("GO forward")
    GPIO.output(Motor1E,GPIO.HIGH)
    forward.ChangeDutyCycle(100)
    reverse.ChangeDutyCycle(0)
    sleep(5)

    #stop motor
    print ("Now stop")
    GPIO.output(Motor1E,GPIO.LOW)
    forward.stop() # stop PWM from GPIO output it is necessary
    reverse.stop() 

    GPIO.cleanup()