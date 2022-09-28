#define stepper motor connections 
#(gpio 5 6 13 19) pin 29,31, 33,35
import RPi.GPIO as GPIO
import time 

#GPIO numbers not pin numbers
#motor 1
dir_1_pin = 5
step_1_pin = 6

#motor 2
dir_2_pin = 13
step_2_pin = 19

stepRev = 400

def main():
    time.sleep(500/1000) # delay(500)
    valveState = input("open valve? yes = 1 ")

    GPIO.setmode(GPIO.BCM) # GPIO.setmode(GPIO.BOARD) refes to pin numbers not GPIO portnumber
    GPIO.setup(dir_1_pin, OUTPUT)
    GPIO.setup(step_1_pin, OUTPUT)
    GPIO.setup(dir_2_pin, OUTPUT)
    GPIO.setup(step_2_pin, OUTPUT)

    bucketL = 70; #[L]
    # parameters to work out:
    #   litres lost in transition phase of stepper
    #   flow rate
    # say flow rat2 is 10 litres per second, you enter x amount of litre 
    #


    while bucketL > 0:
        valveState = input("open valve? yes = 1 ")
        #orderedL = input("Hi can I take your water quantity [L] order? ")
        if valveState == '1':
                #set up for clockwise
                GPIO.Output(dir_1_pin,GPIO.LOW)
                GPIO.Output(dir_2_pin,GPIO.HIGH)
                i = 0
                for i in range (0.25*stepRev):
                    i += 1
                    GPIO.Output(step_1_pin, GPIO.HIGH)
                    GPIO.Output(step_2_pin, GPIO.HIGH)
                    time.sleep(200/1000000) # delay 200 microseconds
                    GPIO.Output(step_1_pin, GPIO.LOW)
                    GPIO.Output(step_2_pin, GPIO.LOW)
                    time.sleep(200/1000000) #delay 200 microseconds
                time.sleep(500/1000)
                #time.sleep(orderedL/flowRt - transition losses)
                #set up for counter clock wise 
                GPIO.Output(dir_1_pin,GPIO.HIGH)
                GPIO.Output(dir_2_pin,GPIO.LOW)
                i = 0
                for i in range (0.25*stepRev):
                    i += 1
                    GPIO.Output(step_1_pin, GPIO.HIGH)
                    GPIO.Output(step_2_pin, GPIO.HIGH)
                    time.sleep(200/1000000) # delay 200 microseconds
                    GPIO.Output(step_1_pin, GPIO.LOW)
                    GPIO.Output(step_2_pin, GPIO.LOW)
                    time.sleep(200/1000000) #delay 200 microseconds
        #bucketL = bucketL - orderedL




                