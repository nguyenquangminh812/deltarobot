import cv2
import numpy as np
import serial
import time
import serial.tools.list_ports

cam = cv2.VideoCapture(0)
ser1 = serial.Serial('/dev/ttyUSB0',115200, timeout = 1)
ser1.reset_input_buffer()

while True:
    ret,image = cam.read()
    img = cv2.medianBlur(image, 3)
    # parameter of image
    width = img.shape[1]
    height = img.shape[0]
    origin = (int(width/2),int(height/2))
    # Convert to grayscale.
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = cv2.inRange(hsv_img,np.array([0, 100, 100]), np.array([10, 255, 255]))
    upper_red = cv2.inRange(hsv_img,np.array([160, 100, 100]), np.array([179, 255, 255]))
    red_image = cv2.addWeighted(lower_red,1.0,upper_red,1.0,0.0)
    red_image = cv2.GaussianBlur(red_image, (9, 9), 2, 2)
    circles = cv2.HoughCircles(red_image, cv2.HOUGH_GRADIENT, 1, red_image.shape[0] / 8, param1=100, param2=18, minRadius=5, maxRadius=60)
    if circles is not None:
  
        # Convert the circle parameters a, b and r to integers.
        circles = np.uint16(np.around(circles))
  
        for pt in circles[0, :]:
            a, b, r = pt[0], pt[1], pt[2]
            
            # draw point coordinate
            cv2.circle(img, origin, 5, (255, 0, 0), -1)
  
            # Draw the circumference of the circle.
            cv2.circle(img, (a, b), r, (0, 255, 0), 2)
        
            #position object in image 
            cv2.putText(img,str(a-origin[0])+","+str(b-origin[1]),(a+10,b+10),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,255),2,cv2.LINE_AA)
  
            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(img, (a, b), 1, (0, 0, 255), 3)
            cv2.imshow('Imaetest',img)
    k = cv2.waitKey(1)
    if k != -1:
        break

cv2.imwrite('/home/Desktop/delta_robot/Camera/testimage.jpg',img)
cam.release()
cv2.destroyAllWindows()
