import RPi.GPIO as GPIO                    #Import GPIO library
import time
import cv2
import numpy as np
import math
# import picamera
# import picamera.array

#Import time library
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)                    # programming the GPIO by BCM pin numbers
TRIG = 17
ECHO = 27
led = 22
m11=16
m12=12
m21=21
m22=20
GPIO.setup(TRIG,GPIO.OUT)                  # initialize GPIO Pin as outputs
GPIO.setup(ECHO,GPIO.IN)                   # initialize GPIO Pin as input
GPIO.setup(led,GPIO.OUT)                  
GPIO.setup(m11,GPIO.OUT)
GPIO.setup(m12,GPIO.OUT)
GPIO.setup(m21,GPIO.OUT)
GPIO.setup(m22,GPIO.OUT)
GPIO.output(led, 1)

threshold1 = 85
threshold2 = 85
theta=0
r_width = 500
r_height = 300
minLineLength = 5
maxLineGap = 10
k_width = 5
k_height = 5
max_slider = 10

time.sleep(5)

def stop():
    GPIO.output(m11, 0)
    GPIO.output(m12, 0)
    GPIO.output(m21, 0)
    GPIO.output(m22, 0)

def forward():
    GPIO.output(m11, 1)
    GPIO.output(m12, 0)
    GPIO.output(m21, 1)
    GPIO.output(m22, 0)

def left():
    GPIO.output(m11, 0)
    GPIO.output(m12, 0)
    GPIO.output(m21, 1)
    GPIO.output(m22, 0)

def right():
    GPIO.output(m11, 1)
    GPIO.output(m12, 0)
    GPIO.output(m21, 0)
    GPIO.output(m22, 0)

stop()
count=0
cam = cv2.VideoCapture(0)
img_counter = 0

while True:
 i=0
 avgDistance=0
 for i in range(5):
  GPIO.output(TRIG, False)                 #Set TRIG as LOW
  time.sleep(0.1)                                   #Delay
  GPIO.output(TRIG, True)                  #Set TRIG as HIGH
  time.sleep(0.00001)                           #Delay of 0.00001 seconds
  GPIO.output(TRIG, False)                 #Set TRIG as LOW

  while GPIO.input(ECHO)==0:              #Check whether the ECHO is LOW
       GPIO.output(led, False)             
  pulse_start = time.time()

  while GPIO.input(ECHO)==1:              #Check whether the ECHO is HIGH
       GPIO.output(led, False) 

  pulse_end = time.time()
  pulse_duration = pulse_end - pulse_start #time to get back the pulse to sensor
  distance = pulse_duration * 17150        #Multiply pulse duration by 17150 (34300/2) to get distance
  distance = round(distance,2)                 #Round to two decimal points
  avgDistance=avgDistance+distance

 avgDistance=avgDistance/5
 print (avgDistance)
 flag=0
 if avgDistance < 15:      #Check whether the distance is within 15 cm range
    count=count+1
    stop()
    time.sleep(1)

 else:
    flag=0
    ret, frame = cam.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # blurred = cv2.GaussianBlur(gray, (k_width, k_height), 0)
    bilateral = cv2.bilateralFilter(gray, 10, 75, 75)
    edged = cv2.Canny(bilateral, threshold1, threshold2)
    lines = cv2.HoughLinesP(edged,1,np.pi/180,max_slider,minLineLength,maxLineGap)
    for x in range(0, len(lines)):
        for x1,y1,x2,y2 in lines[x]:
            cv2.line(frame,(x1,y1),(x2,y2),(255,0,0),3)
            theta=theta+math.atan2((y2-y1),(x2-x1))
            threshold=5
    if(theta>threshold):
       left()
       print("Go left")
       time.sleep(1)
    if(theta<-threshold):
        right()
        print("Go right")
        time.sleep(1)
    if(abs(theta)<threshold):
        forward()
        print("Go straight")
        time.sleep(1)
    theta=0
    if not ret:
        print("failed to grab frame")
        break
    cv2.imshow("test", gray)
    cv2.imshow("test1", bilateral)
    cv2.imshow("test2", edged)
    k = cv2.waitKey(1)
    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break

cam.release()
cv2.destroyAllWindows()