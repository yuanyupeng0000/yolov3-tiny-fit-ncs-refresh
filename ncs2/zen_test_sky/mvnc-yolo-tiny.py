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
    '''
    duration = time.time() - START_TIME
    if(duration >= 120):
        os.system("pkill zenith")
    '''
    results = detector[idx].Detect(picture)
    objs = []
    detectedNum = len(results)
    print("results="+ str(len(results)))
    if detectedNum > 0:
            for i in range(detectedNum):
                classID = results[i].objType
                left = results[i].left
                top = results[i].top
                right = results[i].right
                bottom = results[i].bottom
                confidence = results[i].confidence
                if(results[i].name == 'lpr'):
                    lprtext = results[i].lprtext
                    objs=objs+[classID,confidence,left,top,right-left+1,bottom-top+1, lprtext]
                else:
                    objs=objs+[classID,confidence,left,top,right-left+1,bottom-top+1]
    print(objs)
    return objs

def init1():
    global detector
    gf="graph_file/20190116093312_TinyYoloV3NCS.graph_416"
    detector = ObjectWrapper(gf)
    return detector.devNum

detector = []
NUM = 1
model_xml = 'FP16/20190826111508_TinyYoloV3NCS.xml'
model_bin = 'FP16/20190826111508_TinyYoloV3NCS.bin'
#model_xml = 'FP16/vehicle-license-plate-detection-barrier-0106.xml'
#model_bin = 'FP16/vehicle-license-plate-detection-barrier-0106.bin'
def init(): 
    xml_exists = 'model_xml' in locals() or 'model_xml' in globals()
    bin_exists = 'model_xml' in locals() or 'model_xml' in globals()
    if not (xml_exists and bin_exists and os.path.isfile(model_xml) and os.path.isfile(model_bin)):
        print('NCS: xml or bin file not exists.')
        os.system('pkill zenith')
    for i in range(NUM):
        detector.append(ObjectWrapper(model_xml, model_bin, i))
    return NUM
