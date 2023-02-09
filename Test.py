import cv2
import numpy as np
import math
# import picamera
# import picamera.array

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

cam = cv2.VideoCapture(0)

img_counter = 0

while True:
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

       print("Go left")
    if(theta<-threshold):

        print("Go right")
    if(abs(theta)<threshold):

        print("Go straight")
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