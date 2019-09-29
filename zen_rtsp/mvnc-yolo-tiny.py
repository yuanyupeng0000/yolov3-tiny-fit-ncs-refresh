import sys,os,time,csv,getopt,cv2,argparse
import ctypes
from PIL import Image
import numpy as np
from datetime import datetime
from ObjectWrapper import *
START_TIME = time.time()
def process1(picture, width, height,idx):

    #picture : height*width*3 (BGR)   3D array   
    #image_to_classify = np.zeros((height, width, 3))
    
    #results = detector.Detect(picture,idx)
    #re_in_time_point = time.time()
    #print('re_in_time_point:{0}'.format(re_in_time_point))
    
    duration = time.time() - START_TIME
    #N = N + 1
    #print("N:{0}, fps:{1}".format(N, N/duration))
    results = Detector.Detect(picture, idx)
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

def init1():
    global detector
    gf="graph_file/20190116093312_TinyYoloV3NCS.graph_416"
    detector = ObjectWrapper(gf)
    return detector.devNum
NUM = 1
def init():
    global Detector 
    model_xml = 'FP16/20190411091541_TinyYoloV3NCS.xml' #FP16/20190715102753_TinyYoloV3NCS.xml'
    model_bin = 'FP16/20190411091541_TinyYoloV3NCS.bin'
    Detector = ObjectWrapper(model_xml, model_bin)
    return NUM
