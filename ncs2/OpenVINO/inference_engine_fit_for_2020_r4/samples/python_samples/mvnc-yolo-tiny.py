import sys,os,time,csv,getopt,cv2,argparse
import ctypes
from PIL import Image
import numpy as np
from datetime import datetime
from ObjectWrapper import *

def pro():
    image_to_classify = cv2.imread('195_0.jpg')
    cv2.imshow('demo', mat=image_to_classify)
    cv2.waitKey(0)
    cv2.imwrite('test.jpg',image_to_classify)
    detector = ObjectWrapper('graph_file/yolov2tiny.graph')
    results = detector.Detect(img=image_to_classify)
    print("results: ")
    for j in range(len(results)):
        print(results[j].left)
        print(results[j].top)
        print(results[j].right)
        print(results[j].bottom)
        print(results[j].confidence)
#pro()
def process(picture,width,height):
    print('---into process---')
    '''
    #change BGR to RGB
    image_to_classify = np.zeros((height, width, 3))
    image_to_classify[:height, :width, 2] = picture[:height, :width, 0]
    image_to_classify[:height, :width, 1] = picture[:height, :width, 1]
    image_to_classify[:height, :width, 0] = picture[:height, :width, 2]

    #img = cv2.imread('195_0.jpg') #cv2 can not work
    #cv2.imwrite('test.jpg',img)
    img = Image.fromarray(np.uint8(image_to_classify))
    img.save('pilsave.jpg')

    #results = detector.Detect(image_to_classify)
    '''
    objs = [1,2]
    return objs

def process1(picture, width, height,idx):

    #picture : height*width*3 (BGR)   3D array   
    #image_to_classify = np.zeros((height, width, 3))
    
    results = detector.Detect(picture,idx)

    objs = []
    detectedNum = len(results)
    print("results="+ str(len(results)))
    if detectedNum > 0:
            for i in range(detectedNum):
                classID = results[i].objType #fit for ssd program
                left = results[i].left
                top = results[i].top
                right = results[i].right
                bottom = results[i].bottom
                confidence = results[i].confidence
                objs=objs+[classID,confidence,left,top,right-left+1,bottom-top+1]
    return objs

def init():
    global detector
    gf="graph_file/20181130_person_iter63000.graph_416"
    detector = ObjectWrapper(gf)
    return detector.devNum
