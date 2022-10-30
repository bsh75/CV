#HC-SR04 ultrasonic sensor program for water level indication of wildfire attachment 
# Last edited: 14/10/2022
# Creator: Alfie Mansfield
# 
# Progress update: 
#Sensor providing accurate water level heights in belly tank.
# #Interpolation of water volumes at different heights to still be implemented for accurate measuremtn of water quantity


import statistics
import time
import RPi.GPIO as GPIO

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

# Define GPIO to use on Pi
GPIO_TRIGGER = 27
GPIO_ECHO    = 22
bucketDepth = 30 # cm
height_buffer = [] # buffer to median filter water level
buffer_size = 9

def waterHeight(GPIO_TRIGGER, GPIO_ECHO,bucketDepth):
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(GPIO_TRIGGER,GPIO.OUT)  # Trigger
  GPIO.setup(GPIO_ECHO,GPIO.IN)      # Echo

  # Set trigger to False (Low)
  GPIO.output(GPIO_TRIGGER, False)

  # Allow module to settle
  time.sleep(0.5)

  # Send 10us pulse to trigger
  GPIO.output(GPIO_TRIGGER, True)
  time.sleep(0.00001)
  GPIO.output(GPIO_TRIGGER, False)
  start = time.time()

  while GPIO.input(GPIO_ECHO)==0:
    start = time.time()

  while GPIO.input(GPIO_ECHO)==1:
    stop = time.time()

  # Calculate pulse length
  elapsed = stop-start

  # Distance pulse travelled in that time is time
  # multiplied by the speed of sound (cm/s)
  distance = elapsed * 34300

  # That was the distance there and back so halve the value
  distance = distance / 2
  # subtract bucket Depth for water level
  water_height = bucketDepth - distance

  GPIO.cleanup()
  return water_height


def median_average(height_buffer, size):
    #fill buffer
    while len(height_buffer) < size:
         height_buffer.append(waterHeight(GPIO_TRIGGER, GPIO_ECHO,bucketDepth)) #appending calculated ultrasonic sensor readings to buffer
         
    height_buffer.pop(0) # remove oldest sesnor reding
    height_buffer.append(waterHeight(GPIO_TRIGGER, GPIO_ECHO,bucketDepth)) # append newest sensor reading
   
    median_height = statistics.median(height_buffer) #find median of sensor readings in buffer
    print ("Distance : {:.1f}".format(median_height))
    return median_height

#check refill status
def checkRefill(water_level):
   
    if water_level < 5:
        print("schedule refill Flight path") # when implemented into SA200, queue refill routine here
    else:
        pass


while True:
    #median_average(height_buffer)
    water_level = median_average(height_buffer, buffer_size)
    checkRefill(water_level)