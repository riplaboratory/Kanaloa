import cv2
import numpy as np

def plot(image, name):
	cv2.namedWindow(name,cv2.WINDOW_NORMAL)
	cv2.resizeWindow(name, 800,800)
	cv2.imshow(name,image)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

normal_img = cv2.imread('test_images/shapes.png')

img = cv2.imread('test_images/shapes.png',0)

plot(img, 'Read in Image')

img = cv2.medianBlur(img,5)

plot(img, 'Blurred Image')

cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

plot(normal_img, 'Gray to BGR')

circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=20,maxRadius=80)

circles = np.uint16(np.around(circles))
for i in circles[0,:]:
    # draw the outer circle
    cv2.circle(normal_img,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv2.circle(normal_img,(i[0],i[1]),2,(0,0,255),3)

# print(circles)
# for i in range(3):
# 	# draw the outer circle
# 	cv2.circle(cimg,(circles[i][0],circles[i][1]),circles[i][2],(0,255,0),2)
# 	# draw the center of the circle
# 	cv2.circle(cimg,(circles[i][0],circles[i][1]),2,(0,0,255),3)

plot(normal_img, 'Final Result')


