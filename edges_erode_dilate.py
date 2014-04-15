import cv2
import numpy as np

capture1 = cv2.VideoCapture('amanda.wmv')
ret, img = capture1.read()
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

while True:
    ret, img = capture1.read()
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    #Edge
    detected_edges = cv2.GaussianBlur(gray,(3,3),0)
    detected_edges = cv2.Canny(detected_edges,37,111,apertureSize = 3)
    #Erode
    erosion_size = 5
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(erosion_size,erosion_size))
    eroded = cv2.erode(gray,kernel)
    #Dilate
    dilation_size = 7
    kernal =  cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(dilation_size,dilation_size))
    dilated = cv2.dilate(eroded,kernal)
    #Show videos
    cv2.imshow('edges',detected_edges)
    cv2.imshow('erode & dilate',dilated)
    k = cv2.waitKey(5)
    if k == 27:
        break
cv2.destroyAllWindows()
cv2.VideoCapture('amanda.wmv').release()

