print("initilization ongoing wait for some seconds")

import face_recognition
import cv2
import numpy as np
import os
import smtplib
import imghdr
from email.message import EmailMessage
import RPi.GPIO as GPIO     # Import Library to access GPIO PIN
import time                 # To access delay function

GPIO.setmode(GPIO.BOARD)    # Consider complete raspberry-pi board
GPIO.setwarnings(False)     # To avoid same PIN use warning
# Define GPIO to LCD mapping
LCD_RS = 7
LCD_E  = 11
LCD_D4 = 12
LCD_D5 = 13
LCD_D6 = 15
LCD_D7 = 16
buzzer_pin =36
motor_pin1 =31
motor_pin2 =32
LED_PIN = 29
switch_pin =33
pir_pin = 18

GPIO.setup(LCD_E, GPIO.OUT)  # E
GPIO.setup(LCD_RS, GPIO.OUT) # RS
GPIO.setup(LCD_D4, GPIO.OUT) # DB4
GPIO.setup(LCD_D5, GPIO.OUT) # DB5
GPIO.setup(LCD_D6, GPIO.OUT) # DB6
GPIO.setup(LCD_D7, GPIO.OUT) # DB7
GPIO.setup(LED_PIN,GPIO.OUT)   # Set pin function as output
GPIO.setup(motor_pin1, GPIO.OUT) 
GPIO.setup(motor_pin2, GPIO.OUT) 
GPIO.setup(buzzer_pin,GPIO.OUT)   # Set pin function as output
GPIO.setup(switch_pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)   # Set pin function as input
GPIO.setup(pir_pin,GPIO.IN)   # Set pin function as input  



CurrentFolder = os.getcwd() #Read current folder path
image = "/home/pi/Raspberrypi_workshop/Face_Detection_System/Rahul.jpg"
image2 = "/home/pi/Raspberrypi_workshop/Face_Detection_System/Pranali.jpg"

# This is a demo of running face recognition on live video from your webcam. It's a little more complicated than the
# other example, but it includes some basic performance tweaks to make things run a lot faster:
#   1. Process each video frame at 1/4 resolution (though still display it at full resolution)
#   2. Only detect faces in every other frame of video.

# PLEASE NOTE: This example requires OpenCV (the `cv2` library) to be installed only to read from your webcam.
# OpenCV is not required to use the face_recognition library. It's only required if you want to run this
# specific demo. If you have trouble installing it, try any of the other demos that don't require it instead.

# Get a reference to webcam #0 (the default one)
video_capture = cv2.VideoCapture(0)

# Load a sample picture and learn how to recognize it.
Rahul_image = face_recognition.load_image_file(image)
Rahul_face_encoding = face_recognition.face_encodings(Rahul_image)[0]

# Load a second sample picture and learn how to recognize it.
Pranali_image = face_recognition.load_image_file(image2)
Pranali_face_encoding = face_recognition.face_encodings(Pranali_image)[0]

# Create arrays of known face encodings and their names
known_face_encodings = [
    Rahul_face_encoding,
    Pranali_face_encoding
]
known_face_names = [
    "Rahul_Jadhav",
    "Pranali_Jadhav"
]

#variable to store status of person inside home or not
inside_home = 0
door_open_status =0
# Initialize some variables
face_locations = []
face_encodings = []
face_names = []
process_this_frame = True


# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005
delay = 1


# Define some device constants
LCD_WIDTH = 16    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False
LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

'''
Function Name :lcd_init()
Function Description : this function is used to initialized lcd by sending the different commands
'''
def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)
'''
Function Name :lcd_byte(bits ,mode)
Fuction Name :the main purpose of this function to convert the byte data into bit and send to lcd port
'''
def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command
 
  GPIO.output(LCD_RS, mode) # RS
 
  # High bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x10==0x10:
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)
 
  # Toggle 'Enable' pin
  lcd_toggle_enable()
 
  # Low bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x01==0x01:
    GPIO.output(LCD_D4, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D5, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D6, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D7, True)
 
  # Toggle 'Enable' pin
  lcd_toggle_enable()
'''
Function Name : lcd_toggle_enable()
Function Description:basically this is used to toggle Enable pin
'''
def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)
'''
Function Name :lcd_string(message,line)
Function  Description :print the data on lcd 
'''
def lcd_string(message,line):
  # Send string to display
 
  message = message.ljust(LCD_WIDTH," ")
 
  lcd_byte(line, LCD_CMD)
 
  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

# Define delay between readings
delay = 5
lcd_init()
lcd_string("welcome ",LCD_LINE_1)
time.sleep(1)
lcd_byte(0x01,LCD_CMD) # 000001 Clear display
lcd_string("Face Detection",LCD_LINE_1)
lcd_string("System",LCD_LINE_2)
time.sleep(1)

GPIO.output(LED_PIN,GPIO.LOW)  #LED ON
GPIO.output(buzzer_pin,GPIO.LOW)  #LED ON
GPIO.output(motor_pin1,GPIO.LOW)  #LED ON
GPIO.output(motor_pin2,GPIO.LOW)  #LED ON  
while True:
    #take input from switch
    door_open_status = 0
    lcd_byte(0x01,LCD_CMD) # 000001 Clear display
    lcd_string("Press the Bell",LCD_LINE_1)
    if GPIO.input(switch_pin) == GPIO.HIGH:
        GPIO.output(LED_PIN,GPIO.HIGH)  #LED ON
        GPIO.output(buzzer_pin,GPIO.HIGH)  #LED ON
        time.sleep(2)
        GPIO.output(LED_PIN,GPIO.LOW)  #LED ON
        GPIO.output(buzzer_pin,GPIO.LOW)  #LED ON
        while(1):
            if GPIO.input(PIR_PIN) == GPIO.HIGH:
                 #someone inside home
                 print("someone inside home")
                 inside_home = 1                
            else :
                 #noone inside home
                 print("nobody inside home")
                 inside_home = 0         
            # Grab a single frame of video
            ret, frame = video_capture.read()

            # Resize frame of video to 1/4 size for faster face recognition processing
            small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

            # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
            rgb_small_frame = small_frame[:, :, ::-1]

            # Only process every other frame of video to save time
            if process_this_frame:
                # Find all the faces and face encodings in the current frame of video
                face_locations = face_recognition.face_locations(rgb_small_frame)
                face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

                face_names = []
                for face_encoding in face_encodings:
                    # See if the face is a match for the known face(s)
                    matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                    name = "Unknown"

                    # # If a match was found in known_face_encodings, just use the first one.
                    # if True in matches:
                    #     first_match_index = matches.index(True)
                    #     name = known_face_names[first_match_index]

                    # Or instead, use the known face with the smallest distance to the new face
                    face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                    best_match_index = np.argmin(face_distances)
                    if matches[best_match_index]:
                        name = known_face_names[best_match_index]
                    face_names.append(name)
                    if(name == "Unknown"):
                        #if someone inside home then do not send image on email
                        if(inside_home == 0):
                            i = 0
                            while i < 10:
                                print("sending image on mail")
                                #return_value, image = video_capture.read()
                                cv2.imwrite('opencv.png', frame)
                                i += 1
                                Sender_Email = "jadhav.rahuljadhav.rahul701@gmail.com"
                                Reciever_Email = "rahulraspberrypi@gmail.com"
                                Password = "*******" #type your password here
                                newMessage = EmailMessage()                         
                                newMessage['Subject'] = "Visitor Image" 
                                newMessage['From'] = Sender_Email                   
                                newMessage['To'] = Reciever_Email                   
                                newMessage.set_content('Let me know what you think. Image attached!') 
                                with open('opencv.png', 'rb') as f:
                                    image_data = f.read()
                                    image_type = imghdr.what(f.name)
                                    image_name = f.name
                                newMessage.add_attachment(image_data, maintype='image', subtype=image_type, filename=image_name)
                                with smtplib.SMTP_SSL('smtp.gmail.com', 465) as smtp:
                                    smtp.login(Sender_Email, Password)              
                                    smtp.send_message(newMessage)
                    else:
                        lcd_byte(0x01,LCD_CMD) # 000001 Clear display
                        lcd_string("Welcome",LCD_LINE_1)
                        lcd_string(name,LCD_LINE_2)
                        if(door_open_status == 0):
                            door_open_status = 1
                            GPIO.output(motor_pin1,GPIO.LOW)  #LED ON
                            GPIO.output(motor_pin2,GPIO.HIGH)  #LED ON
                            time.sleep(0.3)
                            GPIO.output(motor_pin1,GPIO.LOW)  #LED ON
                            GPIO.output(motor_pin2,GPIO.LOW)  #LED ON
                            time.sleep(2)
                            GPIO.output(motor_pin1,GPIO.HIGH)  #LED ON
                            GPIO.output(motor_pin2,GPIO.LOW)  #LED ON
                            time.sleep(0.3)
                            GPIO.output(motor_pin1,GPIO.LOW)  #LED ON
                            GPIO.output(motor_pin2,GPIO.LOW)  #LED ON                        
                            time.sleep(1)
                                    
            process_this_frame = not process_this_frame

            # Display the results
            for (top, right, bottom, left), name in zip(face_locations, face_names):
                # Scale back up face locations since the frame we detected in was scaled to 1/4 size
                top *= 4
                right *= 4
                bottom *= 4
                left *= 4

                # Draw a box around the face
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

                # Draw a label with a name below the face
                cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                font = cv2.FONT_HERSHEY_DUPLEX
                cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

            # Display the resulting image
            cv2.imshow('Video', frame)
            if (cv2.waitKey(2) == 27):
                cv2.destroyAllWindows()
                break


# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()