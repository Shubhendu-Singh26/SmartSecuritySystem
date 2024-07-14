import RPi.GPIO as GPIO     # Import Library to access GPIO PIN
import time                 # To access delay function
GPIO.setmode(GPIO.BOARD)    # Consider complete raspberry-pi board
GPIO.setwarnings(False)     # To avoid same PIN use warning
Relay_PIN = 37                # Define PIN for Relay
PIR_PIN = 29                # Define PIN for PIR Sensor
GPIO.setup(Relay_PIN,GPIO.OUT)   # Set pin function as output
GPIO.setup(PIR_PIN,GPIO.IN)   # Set pin function as input  
while (1):
    if GPIO.input(PIR_PIN) == GPIO.HIGH:
        print ("Bulb ON")
        GPIO.output(Relay_PIN,GPIO.HIGH)  #Relay ON                 
    else :
        print ("Bulb OFF")
        GPIO.output(Relay_PIN,GPIO.LOW)   # Relay OFF
