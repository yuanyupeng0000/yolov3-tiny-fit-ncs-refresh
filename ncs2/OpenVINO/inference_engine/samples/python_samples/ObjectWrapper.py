#!/usr/bin/python
# -*- coding: UTF-8 -*-

from libpydetector import YoloDetector
import os, io, numpy, time
import numpy as np
from mvnc import mvncapi as mvnc
#from skimage.transform import resize
import cv2
labels = ["bus","car", "truck", "motorbike", "bicycle","person"]
class BBox(object):
    def __init__(self, bbox, xscale, yscale, offx, offy):
        self.left = int(bbox.left / xscale)-offx
        self.top = int(bbox.top / yscale)-offy
        self.right = int(bbox.right / xscale)-offx
        self.bottom = int(bbox.bottom / yscale)-offy
        self.confidence = bbox.confidence
        self.objType = bbox.objType
        self.name = bbox.name
class BBox_(object):
    def __init__(self, bbox_list, xscale, yscale, offx, offy):
        self.left = int(bbox_list[0] / xscale)-offx
        self.top = int(bbox_list[1] / yscale)-offy
        self.right = int(bbox_list[2] / xscale)-offx
        self.bottom = int(bbox_list[3] / yscale)-offy
        self.confidence = bbox_list[4]
        self.objType = bbox_list[5]
        self.name = labels[bbox_list[5]]

class ObjectWrapper():
    mvnc.global_set_option(mvnc.GlobalOption.RW_LOG_LEVEL, 2)
    devices = mvnc.enumerate_devices()
    devNum = len(devices)
    if len(devices) == 0:
        print('No MVNC devices found')
        quit()
    devHandle = []
    graphHandle = []
    fifoInHandle = []
    fifoOutHandle = []

    def __init__(self, graphfile):
        select = 1
        self.detector = YoloDetector(select)
        for i in range(ObjectWrapper.devNum):
            ObjectWrapper.devHandle.append(mvnc.Device(ObjectWrapper.devices[i])) ##------get devices
            ObjectWrapper.devHandle[i].open() ##------ open device_i
            # load blob
            with open(graphfile, mode='rb') as f:
                blob = f.read()
            # create graph instance
            ObjectWrapper.graphHandle.append(mvnc.Graph('inst' + str(i)))
            # allocate resources
            fifoIn, fifoOut = ObjectWrapper.graphHandle[i].allocate_with_fifos(ObjectWrapper.devHandle[i], blob)
            ObjectWrapper.fifoInHandle.append(fifoIn)
            ObjectWrapper.fifoOutHandle.append(fifoOut)
 
        if(graphfile.endswith('416')):
            self.dim = (416,416)
        elif(graphfile.endswith('288')):
            self.dim = (288,288)
        elif(graphfile.endswith('352')):
            self.dim = (352,352)
        else:
            self.dim = (416, 416)

        self.blockwd = int(self.dim[0]/32)
        self.wh = self.blockwd*self.blockwd
        self.targetBlockwd = int(self.dim[0]/32)
        self.classes = 6
        self.threshold = 0.3
        self.nms = 0.45


    def __del__(self):
        for i in range(ObjectWrapper.devNum):
            ObjectWrapper.fifoInHandle[i].destroy()
            ObjectWrapper.fifoOutHandle[i].destroy()
            ObjectWrapper.graphHandle[i].destroy()
            ObjectWrapper.devHandle[i].close()

    def PrepareImage(self, img, dim):

        '''
        imgw = img.shape[1]
        imgh = img.shape[0]
        imgb = np.empty((dim[0], dim[1], 3))
        imgb.fill(0.5)

        if imgh/imgw > dim[1]/dim[0]:
            neww = int(imgw * dim[1] / imgh)
            newh = dim[1]
        else:
            newh = int(imgh * dim[0] / imgw)
            neww = dim[0]
        offx = int((dim[0] - neww)/2)
        offy = int((dim[1] - newh)/2)

        imgb[offy:offy+newh,offx:offx+neww,:] = resize(img.copy()/255.0,(newh,neww),1)
        im = imgb[:,:,(2,1,0)]
        '''
        
        imgw = img.shape[1]
        imgh = img.shape[0]
        imgb = np.empty((dim[0], dim[1], 3))
        imgb.fill(0.5)

        #neww = 416
        #newh = 416
        neww = dim[0]
        newh = dim[1]

        offx = int((dim[0] - neww)/2)
        offy = int((dim[1] - newh)/2)

        #imgb[offy:offy+newh,offx:offx+neww,:] = resize(img.copy()/255.0,(newh,neww),1)
        imgb[offy:offy+newh,offx:offx+neww,:] = cv2.resize(img/255.0,(newh,neww))
        im = imgb[:,:,(2,1,0)]       

        return im, int(offx*imgw/neww), int(offy*imgh/newh), neww/dim[0], newh/dim[1]
        #return transposed_img, int(offx*imgw/neww), int(offy*imgh/newh), neww/dim[0], newh/dim[1]

    def Reshape(self, out, dim):
        shape = out.shape
        out = np.transpose(out.reshape(self.wh, int(shape[0]/self.wh)))  
        out = out.reshape(shape)
        return out

    def non_max_suppress(self, predicts_dict, threshold=0.3):
        """
        implement non-maximum supression on predict bounding boxes.
        Args:
            predicts_dict: {"stick": [[x1, y1, x2, y2, scores1], [...]]}.
            threshhold: iou threshold
        Return:
            predicts_dict processed by non-maximum suppression
        """
        for object_name, bbox in predicts_dict.items(): #对每一个类别的目标分别进行NMS
            #if(len(bbox)<2):
                #continue
            bbox_array = np.array(bbox, dtype=np.float) ## 获取当前目标类别下所有矩形框（bounding box,下面简称bbx）的坐标和confidence,并计算所有bbx的面积
            #print('bbox_array:{0}'.format(bbox_array))
            x1, y1, x2, y2, scores = bbox_array[:,0], bbox_array[:,1], bbox_array[:,2], bbox_array[:,3], bbox_array[:,4]
            areas = (x2-x1+1) * (y2-y1+1)
            #print "areas shape = ", areas.shape
            ## 对当前类别下所有的bbx的confidence进行从高到低排序（order保存索引信息）
            order = scores.argsort()[::-1]
            #print ("order = {0}".format(order))
            keep = [] #用来存放最终保留的bbx的索引信息 ## 依次从按confidence从高到低遍历bbx，移除所有与该矩形框的IOU值大于threshold的矩形框
            while order.size > 0:
                i = order[0]
                keep.append(i)#保留当前最大confidence对应的bbx索引 ## 获取所有与当前bbx的交集对应的左上角和右下角坐标，并计算IOU（注意这里是同时计算一个bbx与其他所有bbx的IOU）
                xx1 = np.maximum(x1[i], x1[order[1:]])#当order.size=1时，下面的计算结果都为np.array([]),不影响最终结果
                yy1 = np.maximum(y1[i], y1[order[1:]])
                xx2 = np.minimum(x2[i], x2[order[1:]])
                yy2 = np.minimum(y2[i], y2[order[1:]])
                inter = np.maximum(0.0, xx2-xx1+1) * np.maximum(0.0, yy2-yy1+1)
                iou = inter/(areas[i]+areas[order[1:]]-inter)
                #print("iou = {0}".format(iou))
                #print(np.where(iou<=threshold)) #输出没有被移除的bbx索引（相对于iou向量的索引）
                indexs = np.where(iou<=threshold)[0] + 1 #获取保留下来的索引(因为没有计算与自身的IOU，所以索引相差１，需要加上)
                #print ("indexs = {0}".format(type(indexs)))
                order = order[indexs] #更新保留下来的索引
                #print ("order = {0}".format(order))
            bbox = bbox_array[keep]
            predicts_dict[object_name] = bbox.tolist()
            #predicts_dict = predicts_dict
        return predicts_dict

    def non_max_suppress_(self, predicts_dict, nms_tuple=(3, 5), threshold=0.7):
            has_key1 = False
            has_key2 = False
            for key, value in predicts_dict.items():
                if(key == nms_tuple[0]):
                    has_key1 = True
                elif(key == nms_tuple[1]):
                    has_key2 = True
            if((has_key1 == True) and (has_key2 == True)):
                bbx_array = np.array(predicts_dict[nms_tuple[1]], dtype=np.float)
                x1, y1, x2, y2, scores = bbx_array[:,0], bbx_array[:,1], bbx_array[:,2], bbx_array[:,3], bbx_array[:,4]
                areas = (x2-x1+1) * (y2-y1+1)
                keep = []
                for bbx in predicts_dict[nms_tuple[0]]:
                    xx1 = np.maximum(bbx[0], x1)
                    yy1 = np.maximum(bbx[1], y1)
                    xx2 = np.minimum(bbx[2], x2)
                    yy2 = np.minimum(bbx[3], y2)
                    inter = np.maximum(0.0, xx2-xx1+1) * np.maximum(0.0, yy2-yy1+1)
                    iou = inter/((bbx[2]-bbx[0]+1)*(bbx[3]-bbx[1]+1)+areas-inter)
                    print('iou:{0}'.format(iou))
                    print('keep:{0}'.format(np.where(iou<=threshold))) #输出没有被移除的bbx索引（相对于iou向量的索引）
                    indexs = np.where(iou>threshold)[0] #获取保留下来的索引
                    print('keep index:{0}'.format(indexs))
                    keep.append(indexs)
                    print('keep:{0}'.format(keep))
            #bbox = bbox_array[keep]
            #predicts_dict[object_name] = bbox.tolist()
            #predicts_dict = predicts_dict
        #return predicts_dict


    def Detect(self, img, idx=0):
        """Send image for inference on a single compute stick
           
            Args:
                img: openCV image type
                idx: index of the compute stick to use for inference
            Returns:
                [<BBox>]: array of BBox type objects for each result in the detection
        """
        imgw = img.shape[1]
        imgh = img.shape[0]

        im,offx,offy,xscale,yscale = self.PrepareImage(img, self.dim)
        #print('xscale = {}, yscale = {}'.format(xscale, yscale))

        ObjectWrapper.graphHandle[idx].queue_inference_with_fifo_elem(
                ObjectWrapper.fifoInHandle[idx],
                ObjectWrapper.fifoOutHandle[idx],
                im.astype(np.float32), 'user object')
        out, userobj = ObjectWrapper.fifoOutHandle[idx].read_elem()

###################################################################
        '''
        reshaped_out = out.reshape(13, 165, 13)
        transposed_out = np.transpose(reshaped_out, (2, 0, 1))
        '''
        reshaped_out = out.reshape(self.blockwd, 165, self.blockwd)
        transposed_out = np.transpose(reshaped_out, (2, 0, 1))

###################################################################

        transposed_out = transposed_out.reshape(165, self.blockwd, self.blockwd)
        first_132 = transposed_out[:132]
        first_132 = first_132.reshape(33,self.blockwd*2,self.blockwd*2)
        last_33 = transposed_out[132:]
        #print('layer23-conv:\n{0}'.format(first_132))
        #print('layer16-conv:\n{0}'.format(last_33))

###################################################################
        ###out = self.Reshape(out, self.dim)
        out1 = last_33.reshape(self.blockwd*self.blockwd*33)
        internalresults1 = self.detector.Detect(out1.astype(np.float32), 33, self.blockwd, self.blockwd, self.classes, imgw, imgh, self.threshold, self.nms, self.targetBlockwd)
        pyresults1 = [BBox(x,xscale,yscale, offx, offy) for x in internalresults1]

        out2 = first_132.reshape(self.blockwd*2*self.blockwd*2*33)
        internalresults2 = self.detector.Detect(out2.astype(np.float32), 33, self.blockwd*2, self.blockwd*2, self.classes, imgw, imgh, self.threshold, self.nms, self.blockwd*2)
        pyresults2 = [BBox(x,xscale,yscale, offx, offy) for x in internalresults2]
        pyresults3 = pyresults1 + pyresults2

        #return pyresults3

        pre_dic = {}
        list_all = []
        for i in np.arange(6):
            list_temp = []
            for bbx in pyresults3:
                if(bbx.objType == i):
                    list_temp.append([bbx.left, bbx.top, bbx.right, bbx.bottom, bbx.confidence])
                    #print(list_temp)
            if(len(list_temp) == 0):
                continue
            else:
                pre_dic[i] = list_temp
            #--list_all.append(list_temp)
        #--list_key = np.arange(6)
        #--predict_dicts = dict(zip(list_key, list_all))
        #--print('predict_dicts:{0}'.format(predict_dicts))
        #print('pre_dic:{0}'.format(pre_dic))
        nms_pred_dict = self.non_max_suppress(pre_dic)
        #print('nmsed_dict:{0}'.format(nms_pred_dict))
        if(nms_pred_dict == None):
            return []
        ##-------------------------test start
        #self.non_max_suppress_(nms_pred_dict)
        ##-------------------------test end

        nmsed_between_layer_results = []
        for object_id, bboxes in nms_pred_dict.items():
            #print('object_id:{0}'.format(object_id))
            #print('bboxes:{0}'.format(bboxes))
            for bbox in bboxes:
                bbox.append(object_id)
                #print('bbox:{0}'.format(bbox))
                BBox__ = BBox_(bbox, xscale, yscale, offx, offy)
                nmsed_between_layer_results.append(BBox__)

        return nmsed_between_layer_results



        '''out1 = last_33.reshape(13*13*33)
        #internalresults1 = self.detector.Detect(out1.astype(np.float32), 33, self.blockwd, self.blockwd, self.classes, imgw, imgh, self.threshold, self.nms, self.targetBlockwd)
        #pyresults1 = [BBox(x,xscale,yscale, offx, offy) for x in internalresults1]

        out2 = first_132.reshape(26*26*33)
        internalresults2 = self.detector.Detect(out2.astype(np.float32), 33, 26, 26, self.classes, imgw, imgh, self.threshold, self.nms, 26)
        pyresults2 = [BBox(x,xscale,yscale, offx, offy) for x in internalresults2]
        pyresults3 = pyresults1 + pyresults2
        return pyresults3'''

    
    def Parallel(self, img):
        """Send array of images for inference on multiple compute sticks
           
            Args:
                img: array of images to run inference on
           
            Returns:
                { <int>:[<BBox] }: A dict with key-value pairs mapped to compute stick device numbers and arrays of the detection boxs (BBox)
        """
        pyresults = {}
        for i in range(ObjectWrapper.devNum):
            im, offx, offy, w, h = self.PrepareImage(img[i], self.dim)
            ObjectWrapper.graphHandle[i].queue_inference_with_fifo_elem(
                    ObjectWrapper.fifoInHandle[i],
                    ObjectWrapper.fifoOutHandle[i],
                    im.astype(np.float32), 'user object')
        for i in range(ObjectWrapper.devNum):
            out, userobj = ObjectWrapper.fifoOutHandle[i].read_elem()
            out = self.Reshape(out, self.dim)
            imgw = img[i].shape[1]
            imgh = img[i].shape[0]
            internalresults = self.detector.Detect(out.astype(np.float32), int(out.shape[0]/self.wh), self.blockwd, self.blockwd, self.classes, imgw, imgh, self.threshold, self.nms, self.targetBlockwd)
            res = [BBox(x, w, h, offx, offy) for x in internalresults]
            if i not in pyresults:
                pyresults[i] = res
        return pyresults
