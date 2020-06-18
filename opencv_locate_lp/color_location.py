import cv2, sys
import numpy as np
filename = sys.argv[1]
lower_blue = np.array([100, 110, 110])
upper_blue = np.array([130, 255, 255])
lower_yellow = np.array([15, 55, 55])
upper_yellow = np.array([50, 255, 255])
img_src = cv2.imread(filename)
hsv = cv2.cvtColor(img_src, cv2.COLOR_BGR2HSV)
mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
#mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
#output = cv2.bitwise_and(hsv, hsv, mask=mask_blue+mask_yellow)
output = cv2.bitwise_and(hsv, hsv, mask=mask_blue)
# 根据阈值找到对应颜色
output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
Matrix = np.ones((20, 20), np.uint8)
img_edge1 = cv2.morphologyEx(output, cv2.MORPH_CLOSE, Matrix)
img_edge2 = cv2.morphologyEx(img_edge1, cv2.MORPH_OPEN, Matrix)
img, contours, hierarchy = cv2.findContours(img_edge2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
x, y, w, h = cv2.boundingRect(img)
cv2.rectangle(img_src, (x, y), (x+w, y+h), (0, 255, 0), 2)
cv2.imshow("img_src", img_src)
cv2.waitKey(10000)
cv2.destroyAllWindows()
