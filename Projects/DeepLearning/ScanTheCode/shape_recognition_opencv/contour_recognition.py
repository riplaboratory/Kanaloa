import numpy as np
import cv2

# img = cv2.imread('test_images/test_shapes.png')
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# ret,thresh = cv2.threshold(gray,127,255,1)

# # print(cv2.findContours(thresh,1,2))

# contours,h= cv2.findContours(thresh,1,2)

# for cnt in contours:
#     approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
#     print len(approx)
#     if len(approx)==5:
#         print "pentagon"
#         cv2.drawContours(img,[cnt],0,255,-1)
#     elif len(approx)==3:
#         print "triangle"
#         cv2.drawContours(img,[cnt],0,(0,255,0),-1)
#     elif len(approx)==4:
#         print "square"
#         cv2.drawContours(img,[cnt],0,(0,0,255),-1)
#     elif len(approx) == 9:
#         print "half-circle"
#         cv2.drawContours(img,[cnt],0,(255,255,0),-1)
#     elif len(approx) > 15:
#         print "circle"
#         cv2.drawContours(img,[cnt],0,(0,255,255),-1)

# def ShapeDetector()
# # initialize the shape name and approximate the contour

#         shape = "unidentified"
#         peri = cv2.arcLength(c, True)
#         approx = cv2.approxPolyDP(c, 0.04 * peri, True)

#         nCV shape detectionPython

# import the necessary packages
import cv2
import imutils

class ShapeDetector:
    def __init__(self):
        pass

    def detect(self, c):
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)

        # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
            shape = "triangle"
 
        # if the shape has 4 vertices, it is either a square or
        # a rectangle
        elif len(approx) == 4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
 
            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
            shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
 
        # if the shape is a pentagon, it will have 5 vertices
        elif len(approx) == 5:
            shape = "pentagon"
 
        # otherwise, we assume the shape is a circle
        else:
            shape = "circle"
 
        # return the name of the shape
        return shape


# load the image and resize it to a smaller factor so that
# the shapes can be approximated better
image = cv2.imread('test_images/Red_rgb.png')
resized = imutils.resize(image, width=300)
ratio = image.shape[0] / float(resized.shape[0])
 
# convert the resized image to grayscale, blur it slightly,
# and threshold it
gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
 
# find contours in the thresholded image and initialize the
# shape detector
cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if imutils.is_cv2() else cnts[1]
sd = ShapeDetector()


for c in cnts:
    # compute the center of the contour, then detect the name of the
    # shape using only the contour
    M = cv2.moments(c)
    cX = int((M["m10"] / M["m00"]) * ratio)
    cY = int((M["m01"] / M["m00"]) * ratio)
    shape = sd.detect(c)
 
    # multiply the contour (x, y)-coordinates by the resize ratio,
    # then draw the contours and the name of the shape on the image
    c = c.astype("float")
    c *= ratio
    c = c.astype("int")
    cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
    cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (255, 255, 255), 2)
 
    # show the output image
    cv2.namedWindow('Image',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Image', 800,800)
    cv2.imshow("Image", image)
    cv2.waitKey(0)

#     # loop over the contours
# for c in cnts:
#     # compute the center of the contour, then detect the name of the
#     # shape using only the contour
#     M = cv2.moments(c)
#     cX = int((M["m10"] / M["m00"]) * ratio)
#     cY = int((M["m01"] / M["m00"]) * ratio)
#     shape = sd.detect(c)
 
#     # multiply the contour (x, y)-coordinates by the resize ratio,
#     # then draw the contours and the name of the shape on the image
#     c = c.astype("float")
#     c *= ratio
#     c = c.astype("int")
#     cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
#     cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
#         0.5, (255, 255, 255), 2)
 
#     # show the output image
#     cv2.namedWindow('Image',cv2.WINDOW_NORMAL)
#     cv2.resizeWindow('Image', 800,800)
#     cv2.imshow("Image", image)
#     cv2.waitKey(0)


