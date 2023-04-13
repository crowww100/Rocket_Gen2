import cv2


# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time


# initialize the camera and grab a reference to the raw camera capture
#camera = PiCamera()
#rawCapture = PiRGBArray(camera)
# allow the camera to warmup
#time.sleep(0.1)

cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(3,640)
cap.set(4,480)

ret, frame = cap.read()
cv2.imshow("test", frame)
cv2.waitKey(0)

a=0
while 1:

   a= a+1
   ret, frame = cap.read()
   if (ret):
      cv2.imshow("test", frame)
   # grab an image from the camera
   #frame = rawCapture.array
      #print("image captured")


   # show results
   #cv2.imshow("frame", frame)
   #time.sleep(1)

   if cv2.waitKey(1) == ord('q'):
#        p_x.stop()
#        p_y.stop()
#        GPIO.cleanup()
        break

