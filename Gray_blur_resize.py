import cv2

capture1 = cv2.VideoCapture('IMG_2843.MOV')

while True:
	ret, img = capture1.read() # Read an image
	if (type(img) == type(None)):
            break       
	image1 = cv2.resize(img, (0,0),fx=.5, fy=.5)
        cv2.imshow("Resize", image1) # Display the image
        img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
	image2 = cv2.resize(img_gray, (0,0),fx=.75, fy=.75)
        cv2.imshow("Grayscale", image2)        
	blur = cv2.blur(img,(20,20))
	image3 = cv2.resize(blur, (0,0),fx=.75, fy=.75)
        cv2.imshow("Blur", image3)
    	if (0xFF & cv2.waitKey(5) == 27) or img.size == 0:
		break
