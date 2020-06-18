import sys 
import cv2
import numpy as np
import math
import os
 
def stretch(img):
    max = float(img.max())
    min = float(img.min())
 
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            img[i, j] = (255/(max-min))*img[i,j]-(255*min)/(max-min)  ##[255/(max-min)]*(img[i,j]-min)
             
    return img
     
def dobinaryzation(img):
    max = float(img.max())
    min = float(img.min())
     
    x = max - ((max-min) / 2)
    ret, threshedimg = cv2.threshold(img, x, 255, cv2.THRESH_BINARY)
     
    return threshedimg
 
def find_retangle(contour):
	y, x = [], []
	
	for p in contour:
		y.append(p[0][0])
		x.append(p[0][1])
		
	return [min(y), min(x), max(y), max(x)]
 
def locate_license(img, orgimg):
	img, contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	###img, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	#x, y, w, h = cv2.boundingRect(img)
	#cv2.rectangle(orgimg, (x, y), (x+w, y+h), (0, 255, 0), 2)
	#cv2.drawContours(orgimg,contours,-1,(0,0,255),3)
	#cv2.imwrite('test.jpg', orgimg)
 
	# 找出最大的三个区域
	blocks = []
	epsilo = 0.00001
	for c in contours:
		# 找出轮廓的左上点和右下点，由此计算它的面积和长宽比
		r = find_retangle(c)
		a = (r[2]-r[0]) * (r[3]-r[1])
		s = (r[2]-r[0]) / (r[3]-r[1] + epsilo)
		if(s > 100 or s < 0.01):
			continue
		
		blocks.append([r, a, s])
		
	# 选出面积最大的3个区域
	blocks = sorted(blocks, key=lambda b: b[1])[-3:]
	print(blocks)
		
	# 使用颜色识别判断找出最像车牌的区域
	maxweight, maxindex = 0, 0
	for i in range(len(blocks)):
		b = orgimg[blocks[i][0][1]:blocks[i][0][3], blocks[i][0][0]:blocks[i][0][2]]
		# RGB转HSV
		hsv = cv2.cvtColor(b, cv2.COLOR_BGR2HSV)
		# 蓝色车牌范围
		lower = np.array([100,50,50])
		upper = np.array([140,255,255])
		# 根据阈值构建掩模
		mask = cv2.inRange(hsv, lower, upper)
 
		# 统计权值
		w1 = 0
		for m in mask:
			w1 += m / 255
		
		w2 = 0
		for w in w1:
			w2 += w
			
		# 选出最大权值的区域
		if w2 > maxweight:
			maxindex = i
			maxweight = w2
	
		
	return blocks[maxindex][0]
	
def find_license(img):
	'''预处理'''
	# 压缩图像
	cv2.imshow("src", img)
	cv2.waitKey(10000000)
	cv2.destroyAllWindows()
	SHRINK = 100
	img = cv2.resize(img, (SHRINK, int(SHRINK*img.shape[0]/img.shape[1])))
	cv2.imshow("resized", img)
	cv2.waitKey(10000000)
	cv2.destroyAllWindows()	 
	# RGB转灰色
	grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	 
	# 灰度拉伸
	stretchedimg = stretch(grayimg)
	 
	# 进行开运算，用来去噪声
	
	###r = 16
	r = 4
	h = w = r * 2 + 1
	kernel = np.zeros((h, w), dtype=np.uint8)
	cv2.circle(kernel, (r, r), r, 1, -1)
	 
	openingimg = cv2.morphologyEx(stretchedimg, cv2.MORPH_OPEN, kernel)
	#cv2.imshow("opening1 img", openingimg)
	#cv2.waitKey(100000)
	#cv2.destroyAllWindows()
	strtimg = cv2.absdiff(stretchedimg, openingimg)
	#cv2.imshow("absdiff", strtimg)
	#cv2.waitKey(100000)
	#cv2.destroyAllWindows()
		 
	# 图像二值化
	###binaryimg = dobinaryzation(strtimg)
	binaryimg = dobinaryzation(stretchedimg)
	#cv2.imshow("binaryimg", binaryimg)
	#cv2.waitKey(100000)
	#cv2.destroyAllWindows()	 
	# 使用Canny函数做边缘检测
	cannyimg = cv2.Canny(binaryimg, binaryimg.shape[0], binaryimg.shape[1])
	img1 = cannyimg.copy()
	cv2.imshow("cannyimg", cannyimg)
	cv2.waitKey(10000000)
	cv2.destroyAllWindows()	
	''' 消除小区域，保留大块区域，从而定位车牌'''
	# 进行闭运算
	###kernel = np.ones((5,19), np.uint8)
	kernel = np.ones((5, 5), np.uint8)
	closingimg = cv2.morphologyEx(cannyimg, cv2.MORPH_CLOSE, kernel, iterations = 1)
	cv2.imshow("closingimg1", closingimg)
	cv2.waitKey(100000000)
	cv2.destroyAllWindows()
	#返回图像的高和宽
	(h,w)=closingimg.shape 
	#初始化一个跟图像高一样长度的数组，用于记录每一行的黑点个数
	a=[0 for z in range(0,h)] 
	for i in range(0,h):          #遍历每一行
		for j in range(0,w):      #遍历每一列
			if img1[i,j]==0:      #判断该点是否为黑点，0代表黑点
				a[i]+=1           #该行的计数器加一
				img1[i,j]=255     #将其改为白点，即等于255
	for i in range(0,h):          #遍历每一行
		for j in range(0,a[i]):   #从该行应该变黑的最左边的点开始向最右边的点设置黑点
			img1[i,j]=0           #设置黑点
	cv2.imshow("img1",img1) 
	key = cv2.waitKey(10000000)
	cv2.destroyAllWindows()
	kernel = np.ones((1,9),np.uint8)  
	erosion = cv2.erode(img1,kernel,iterations = 1)
	cv2.imshow("erosion", erosion)
	cv2.waitKey(100000)
	cv2.destroyAllWindows()
	#返回图像的高和宽

	#初始化一个跟图像宽一样长度的数组，用于记录每一列的黑点个数
	a =[0 for z in range(0,w)]
	img2 = closingimg.copy()
	for i in range(0,w):           #遍历每一列
		for j in range(0,h):       #遍历每一行
			if img2[j,i]==0:       #判断该点是否为黑点，0代表是黑点
				a[i]+=1            #该列的计数器加1
				img2[j,i]=255      #记录完后将其变为白色，即等于255
		for i in range(0,w):           #遍历每一列
			for j in range(h-a[i],h):  #从该列应该变黑的最顶部的开始向最底部设为黑点
				img2[j,i]=0            #设为黑点
	cv2.imshow("img2",img2)
	cv2.waitKey(10000000)
	cv2.destroyAllWindows()
	kernel = np.ones((9,1),np.uint8)  
	erosion = cv2.erode(img2,kernel,iterations = 1)
	cv2.imshow("erosion", erosion)
	cv2.waitKey(100000)
	cv2.destroyAllWindows()
	'''
	kernel = np.ones((2,2),np.uint8)  
	erosion = cv2.erode(closingimg,kernel,iterations = 3)
	cv2.imshow("erosion", erosion)
	cv2.waitKey(100000)
	cv2.destroyAllWindows()
	'''
	'''
	line = 100
	minLineLength = 40
	# HoughLinesP函数是概率直线检测，注意区分HoughLines函数
	lines = cv2.HoughLinesP(closingimg, 1, np.pi/180, 50, lines=line, minLineLength=minLineLength)
	# 降维处理
	lines1 = lines[:,0,:]
	# line 函数勾画直线
	# (x1,y1),(x2,y2)坐标位置
	# (0,255,0)设置BGR通道颜色
	# 2 是设置颜色粗浅度
	for x1,y1,x2,y2 in lines1:
		cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
	cv2.imshow("lines", img)
	cv2.waitKey(100000)
	cv2.destroyAllWindows()	
	'''	 
	# 进行闭运算
	###kernel = np.ones((5,19), np.uint8)
	kernel = np.ones((5,19), np.uint8)
	closingimg = cv2.morphologyEx(closingimg, cv2.MORPH_CLOSE, kernel)
	cv2.imshow("closingimg2", closingimg)
	cv2.waitKey(100000000)
	cv2.destroyAllWindows()
	# 进行闭运算
	###kernel = np.ones((5,19), np.uint8)
	#kernel = np.ones((5,19), np.uint8)
	#closingimg = cv2.morphologyEx(closingimg, cv2.MORPH_CLOSE, kernel)
	#cv2.imshow("closingimg3", closingimg)
	#cv2.waitKey(100000)
	#cv2.destroyAllWindows()
	# 进行闭运算
	###kernel = np.ones((5,19), np.uint8)
	#kernel = np.ones((5,19), np.uint8)
	#closingimg = cv2.morphologyEx(closingimg, cv2.MORPH_CLOSE, kernel)
	#cv2.imshow("closingimg4", closingimg)
	#cv2.waitKey(100000)
	#cv2.destroyAllWindows()
	# 进行闭运算
	###kernel = np.ones((5,19), np.uint8)
	#kernel = np.ones((5,19), np.uint8)
	#closingimg = cv2.morphologyEx(closingimg, cv2.MORPH_CLOSE, kernel)
	#cv2.imshow("closingimg5", closingimg)
	#cv2.waitKey(100000)
	#cv2.destroyAllWindows()
        
	# 进行开运算
	'''
	openingimg = cv2.morphologyEx(closingimg, cv2.MORPH_OPEN, kernel)
	cv2.imshow("opening2 img", openingimg)
	cv2.waitKey(10000)
	cv2.destroyAllWindows()	 
	# 再次进行开运算
	###kernel = np.ones((11,5), np.uint8)	
	kernel = np.ones((11,5), np.uint8)
	openingimg = cv2.morphologyEx(openingimg, cv2.MORPH_OPEN, kernel)	
	cv2.imshow("opening3 img", openingimg)
	cv2.waitKey(10000)
	cv2.destroyAllWindows()
        '''
	
	# 消除小区域，定位车牌位置
	rect = locate_license(closingimg, img)
	###rect = locate_license(cannyimg, img)
	
	return rect, img
	
if __name__ == '__main__':
	# 读取图片
	img_list = []
	img = sys.argv[1]
	preffix = ""
	if(os.path.isfile(img)):
		img_list.append(img)
	elif(os.path.isdir(img)):
		preffix = img
		imgs = os.listdir(img)	
		for im in imgs:
			img_list.append(preffix + "/" + im)
	for i in img_list:
		print(i)
		orgimg = cv2.imread(i)
		rect, img = find_license(orgimg)
 
		# 框出车牌
		cv2.rectangle(img, (rect[0], rect[1]), (rect[2], rect[3]), (0,255,0),2)
		cv2.imshow(i.split('/')[-1], img)
 
		cv2.waitKey(3000)
		cv2.destroyAllWindows()
