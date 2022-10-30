# DS5160 60kg-cm servo motor program to actuate butterfly valve 
#last editted: 14/10/2022
#Creator: Alfie Mansfield  

#Progress update
#Tested as stand alone progrma with manual activation of actuation via a button
#Autonomously actuating servo from having a downward gimbal orientation and hotspot target in centre of frame still to be implemented 

from gpiozero import AngularServo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM) #set mode to refer to GPIO pin numbers 
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP) # set up GPIO23 (pin 16) as button for testing operation
 
factory = PiGPIOFactory()

#initialise servo using gpiozero library
servo =AngularServo(18, min_angle=0, max_angle=270, min_pulse_width=0.0005, max_pulse_width=0.0025,pin_factory = factory)

closePos = 0 # closed valve servo position angle
openPos = 90 # open valve servo position angle 
specified_L = 20 # for integrated system  specified_L = getQuantity()
transition_L = 10 # water quanitiy lost in actuator transition period 
flow_rate  = 15 #flow rate of butterfly valve

drop_L = specified_L-transition_L 
valve_open = drop_L/flow_rate 

servo.angle = closePos


while True:
    if GPIO.input(23) == GPIO.LOW: # press button or connect GPIO23 to ground to actuate valve 
        i = 0
        for i in range(openPos+1):
            servo.angle = i
            sleep(0.005) # delay to allow servo to move to position 
        sleep (valve_open)
        for i in range(openPos+1):
            servo.angle = openPos-i
            sleep(0.005)
    

# function framework for integrated system (not yet tested)
# def dropWater(available_L):
#     servo =AngularServo(18, min_angle=0, max_angle=270, min_pulse_width=0.0005, max_pulse_width=0.0025)

#     closePos = 0 # closed valve servo position angle
#     openPos = 91 # open valve servo position angle 
#     specified_L = 20 # for integrated system  specified_L = getQuantity()
#     transition_L = 10 # water quanitiy lost in actuator transition period 
#     flow_rate  = 15 #flow rate of butterfly valve

#     drop_L = specified_L-transition_L # 
#     valve_open = drop_L/flow_rate

#     servo.angle = closePos #initialise close servo position
# for integrated system insert condition: if gimbalservo1.pos == 0 and gimbalservo2.pos == 0 and targetPixel in centre of frame:
#     i = 0
#     for i in range(openPos+1):
#         servo.angle = i
#         sleep(0.05) # delay to allow serfvo to move to position 
#     sleep (valve_open)
#     for i in range(openPos+1):
#         servo.angle = openPos-i
#         sleep(0.05)
#




    