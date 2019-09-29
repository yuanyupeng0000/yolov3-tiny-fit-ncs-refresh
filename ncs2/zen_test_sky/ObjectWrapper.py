from libpydetector import YoloDetector
import os, sys, io, time
import numpy as np
#from skimage.transform import resize
import cv2
import logging
from openvino.inference_engine import IENetwork, IEPlugin
logging.basicConfig(format="[ %(levelname)s ] %(message)s", level=logging.DEBUG, stream=sys.stdout)
log = logging.getLogger()
labels = ["bus","car", "truck", "motorbike", "bicycle","person", "plate"]
#labels_map = ["backgroud","vehicle","lpr"]
lpr_labels = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9","<Anhui>", "<Beijing>", "<Chongqing>", "<Fujian>","<Gansu>", "<Guangdong>", "<Guangxi>", "<Guizhou>","<Hainan>", "<Hebei>", "<Heilongjiang>", "<Henan>","<HongKong>", "<Hubei>", "<Hunan>", "<InnerMongolia>","<Jiangsu>", "<Jiangxi>", "<Jilin>", "<Liaoning>","<Macau>", "<Ningxia>", "<Qinghai>", "<Shaanxi>","<Shandong>", "<Shanghai>", "<Shanxi>", "<Sichuan>",
"<Tianjin>", "<Tibet>", "<Xinjiang>", "<Yunnan>","<Zhejiang>", "<police>","A", "B", "C", "D", "E", "F", "G", "H", "I", "J","K", "L", "M", "N", "O", "P", "Q", "R", "S", "T","U", "V", "W", "X", "Y", "Z"]

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

class _BBox(object):
    def __init__(self, left, top, right, bottom, confidence, objType, name, lprtext=''):
        #print('left top right bottom confidence :{0},{1},{2},{3},{4}, {5}'.format(left, top, right, bottom, confidence, objType))
        self.left = int(left)
        self.top = int(top)
        self.right = int(right)
        self.bottom = int(bottom)
        self.confidence = confidence
        self.objType = objType
        self.name = name
        if(name == 'lpr'):
            self.lprtext = lprtext


def non_max_suppress(predicts_dict, threshold=0.65):
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
def parse_ncs2_yolov3_tiny_output(output, frame, detector, W_H):
    for layer_name, out_blob in output.items():
        #print(layer_name)
        #print(out_blob.shape)
        blockwd = int(W_H/32)
        classes = int(out_blob.shape[1]/3 - 5)
        imgw = frame.shape[1]
        imgh = frame.shape[0]
        threshold = 0.3
        nms = 0.65
        targetBlockwd = blockwd
        N = (4+1+classes)*3

        #if (layer_name == 'layer10-conv'):
        #print(layer_name)
        if (int(layer_name[5:7]) <= 16):
            out1 = out_blob.reshape(blockwd * blockwd * N)
            internalresults1 = detector.Detect(out1.astype(np.float32), N, blockwd, blockwd, classes, imgw,
                                               imgh, threshold, nms, targetBlockwd)
            pyresults1 = [BBox(x, 1, 1, 0, 0) for x in internalresults1]
        #elif (layer_name == 'layer17-conv'):
        elif (int(layer_name[5:7]) > 16):
            out2 = out_blob.reshape(blockwd * 2 * blockwd * 2 * N)
            internalresults2 = detector.Detect(out2.astype(np.float32), N, blockwd * 2, blockwd * 2, classes,
                                               imgw, imgh, threshold, nms, blockwd * 2)
            pyresults2 = [BBox(x, 1, 1, 0, 0) for x in internalresults2]

    pyresults3 =  pyresults1 + pyresults2
    # print(pyresults3)
    pre_dic = {}
    list_all = []
    for i in np.arange(len(labels)):
        list_temp = []
        for bbx in pyresults3:
            if (bbx.objType == i):
                list_temp.append([bbx.left, bbx.top, bbx.right, bbx.bottom, bbx.confidence])
        if (len(list_temp) == 0):
            continue
        else:
            pre_dic[i] = list_temp
    nms_pred_dict = non_max_suppress(pre_dic)
    if (nms_pred_dict == None):
        return []
    nmsed_between_layer_results = []
    for object_id, bboxes in nms_pred_dict.items():
        for bbox in bboxes:
            bbox.append(object_id)
            BBox__ = BBox_(bbox, 1, 1, 0, 0)
            nmsed_between_layer_results.append(BBox__)

    return nmsed_between_layer_results



class ObjectWrapper():
    args_device = 'MYRIAD'
    args_plugin_dir = None
    num_requests = 1
    args_cpu_extension = False
    
    
    def __init__(self, model_xml, model_bin, index_i):
        select = 1
        self.video_chanel = index_i
        if('TinyYoloV3' in model_xml):
            self.base_detection = 'TinyYoloV3'
        else:
            self.base_detection = 'ssd'   
        # --------------------------------------------- model detect start-------------------------------------------------------#
        self.num_requests = ObjectWrapper.num_requests
        self.cur_request_id = 0
        self.next_request_id = 1
        self.previous_request_id = 1 - ObjectWrapper.num_requests
        self.detector = YoloDetector(select)
        # ------------- 1. Plugin initialization for specified device and load extensions library if specified -------------
        plugin = IEPlugin(device=ObjectWrapper.args_device, plugin_dirs=ObjectWrapper.args_plugin_dir) ###
        if ObjectWrapper.args_cpu_extension and 'CPU' in ObjectWrapper.args_device:
            plugin.add_cpu_extension(ObjectWrapper.args_cpu_extension)

        # -------------------- 2. Reading the IR generated by the Model Optimizer (.xml and .bin files) --------------------
        log.info("Loading network files:\n\t{}\n\t{}".format(model_xml, model_bin))
        net = IENetwork(model=model_xml, weights=model_bin) ###

        # ---------------------------------- 3. Load CPU extension for support specific layer ------------------------------
        if plugin.device == "CPU":
            supported_layers = plugin.get_supported_layers(net)
            not_supported_layers = [l for l in net.layers.keys() if l not in supported_layers]
            if len(not_supported_layers) != 0:
                log.error("Following layers are not supported by the plugin for specified device {}:\n {}".
                      format(plugin.device, ', '.join(not_supported_layers)))
                log.error("Please try to specify cpu extensions library path in sample's command line parameters using -l "
                      "or --cpu_extension command line argument")
                sys.exit(1)

        #assert len(net.inputs.keys()) == 1, "Sample supports only YOLO V3 based single input topologies"
        #assert len(net.outputs) == 2, "Sample supports only YOLO V3 Tiny based double output topologies"

        # ---------------------------------------------- 4. Preparing inputs -----------------------------------------------
        log.info("Preparing inputs")
        input_blob = next(iter(net.inputs))

        #  Defaulf batch_size is 1
        net.batch_size = 1

        # Read and pre-process input images
        self.n, self.c, self.h, self.w = net.inputs[input_blob].shape

        # ----------------------------------------- 5. Loading model to the plugin -----------------------------------------
        log.info("Loading model to the plugin")
        exec_net = plugin.load(network=net, num_requests=ObjectWrapper.num_requests) ###
        print("Load sucess")
        self.exec_net = exec_net
        self.infer_requests = exec_net.requests
        self.plugin = plugin
        self.net = net
        self.input_blob = input_blob
        # --------------------------------------------- model detect end-------------------------------------------------------#
        
        
        # --------------------------------------------- model LPR start-------------------------------------------------------#
        self.num_requests = ObjectWrapper.num_requests
        self.lpr_cur_request_id = 0
        self.lpr_next_request_id = 1
        self.lpr_previous_request_id = 1 - ObjectWrapper.num_requests

        # ------------- 1. Plugin initialization for specified device and load extensions library if specified -------------
        print("IEPlugin ...")
        lpr_plugin = IEPlugin(device=ObjectWrapper.args_device, plugin_dirs=ObjectWrapper.args_plugin_dir) ###
        print("IEPlugin sucess")
        if ObjectWrapper.args_cpu_extension and 'CPU' in ObjectWrapper.args_device:
            lpr_plugin.add_cpu_extension(ObjectWrapper.args_cpu_extension)

        # -------------------- 2. Reading the IR generated by the Model Optimizer (.xml and .bin files) --------------------
        lpr_model_xml = 'FP16/license-plate-recognition-barrier-0001.xml'
        lpr_model_bin = 'FP16/license-plate-recognition-barrier-0001.bin'
        log.info("Loading network files:\n\t{}\n\t{}".format(lpr_model_xml, lpr_model_bin))
        lpr_net = IENetwork(model=lpr_model_xml, weights=lpr_model_bin) ###
        
        # ---------------------------------- 3. Load CPU extension for support specific layer ------------------------------
        if lpr_plugin.device == "CPU":
            supported_layers = lpr_plugin.get_supported_layers(net)
            not_supported_layers = [l for l in lpr_net.layers.keys() if l not in supported_layers]
            if len(not_supported_layers) != 0:
                log.error("Following layers are not supported by the plugin for specified device {}:\n {}".
                      format(lpr_plugin.device, ', '.join(not_supported_layers)))
                log.error("Please try to specify cpu extensions library path in sample's command line parameters using -l "
                      "or --cpu_extension command line argument")
                sys.exit(1)

        #assert len(lpr_net.inputs.keys()) == 1, "Sample supports only YOLO V3 based single input topologies"
        #assert len(lpr_net.outputs) == 2, "Sample supports only YOLO V3 Tiny based double output topologies"
        
        # ---------------------------------------------- 4. Preparing inputs -----------------------------------------------
        log.info("Preparing inputs")
        #lpr_input_blob = next(iter(lpr_net.inputs))
        ### 
        lpr_input_blob_1 = 'data'
        lpr_input_blob_0 = 'seq_ind'
        
        #  Defaulf batch_size is 1
        lpr_net.batch_size = 1
        
        # Read and pre-process input images
        self.lpr_n, self.lpr_c, self.lpr_h, self.lpr_w = lpr_net.inputs['data'].shape
        #print(lpr_net.inputs['seq_ind'].shape)
        
        # ----------------------------------------- 5. Loading model to the plugin -----------------------------------------
        log.info("Loading model to the plugin")
        #lpr_exec_net = lpr_plugin.load(network=lpr_net, num_requests=ObjectWrapper.num_requests) ###
        lpr_exec_net = plugin.load(network=lpr_net, num_requests=ObjectWrapper.num_requests)
        print("Load sucess")
        self.lpr_exec_net = lpr_exec_net
        self.lpr_infer_requests = lpr_exec_net.requests
        self.lpr_plugin = lpr_plugin
        self.lpr_net = lpr_net
        #self.lpr_input_blob = lpr_input_blob
        # --------------------------------------------- model LPR end-------------------------------------------------------#       

    def __del__(self):
        del self.net
        del self.exec_net
        del self.plugin
        del self.infer_requests
        del self.detector
        del self.input_blob
        # --------------------
        
        del self.lpr_net
        del self.lpr_exec_net
        del self.lpr_plugin
        del self.lpr_infer_requests
        #del self.lpr_input_blob

    def parse_ncs2_ssd_output(self, output, frame): #W_H=self.w, video_chanel=self.video_chanel
        for layer_name, out_blob in output.items():
            #print(layer_name)
            #print(out_blob.shape)
            results_ = []
            for i in range(10):
                # Draw only objects when probability more than specified threshold
                if out_blob[0][0][i][2] > 0.1: 
                    xmin = int(out_blob[0][0][i][3] * frame.shape[1])
                    ymin = int(out_blob[0][0][i][4] * frame.shape[0])
                    xmax = int(out_blob[0][0][i][5] * frame.shape[1])
                    ymax = int(out_blob[0][0][i][6] * frame.shape[0])
                    class_id = int(out_blob[0][0][i][1])
                    class_name = 'not_lpr' if class_id == 1 else 'lpr'
                    __BBox = _BBox(xmin, ymin, xmax, ymax, out_blob[0][0][i][2], class_id, class_name)
                    results_.append(__BBox)
                    #Draw box and label\class_id
                    #color = (min(class_id * 12.5, 255), min(class_id * 7, 255), min(class_id * 5, 255))
                    #cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                    #det_label = labels_map[class_id] if labels_map else str(class_id)
                    #cv2.putText(frame, det_label + ' ' + str(round(obj[2] * 100, 1)) + ' %', (xmin, ymin - 7),
                    #            cv2.FONT_HERSHEY_COMPLEX, 0.6, color, 1)
        print('video_chanel:{0}'.format(self.video_chanel))
        #if(self.video_chanel == 0):
        #    cv2.imwrite('test', frame)
        # cv2.waitkey(1)
        return results_


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
    def get_LPR(self, nmsed_between_layer_results, in_frame):
        #print('----------------------------------------get lpr start-----------------------------------')
        seq_ind_ = np.ones(88)
        seq_ind = seq_ind_.reshape((88,1))
        seq_ind[0][0] = 0.0
        
        #for sigle image
        '''
        in_frame = cv2.imread('/video/picture/mini_plate.png')                            
        #in_frame = in_frame.reshape((self.c, self.h, self.w))
        #print(in_frame.shape)
        #in_frame = in_frame.transpose((2, 1, 0))  # Change data layout
        print(in_frame.shape)
        in_frame = cv2.resize(in_frame, (self.lpr_w, self.lpr_h))
        print(in_frame.shape)
        in_frame = in_frame.transpose((2, 0, 1))  # Change data layout
        print(in_frame.shape)
        in_frame = in_frame.reshape((self.lpr_n, self.lpr_c, self.lpr_h, self.lpr_w))
        '''
        
        #print(seq_ind_)
        lpr_result_dic = {}
        for obj in  nmsed_between_layer_results:
            if(obj.objType == 2):
                #for crop
                #print(in_frame.shape)
                in_frame = in_frame[obj.top:obj.bottom,obj.left:obj.right,:]
                #print(in_frame.shape)
                in_frame = cv2.resize(in_frame, (self.lpr_w, self.lpr_h))
                in_frame = in_frame.transpose((2, 0, 1))  # Change data layout
                in_frame = in_frame.reshape((self.lpr_n, self.lpr_c, self.lpr_h, self.lpr_w))
                #print(in_frame.shape)
                self.lpr_exec_net.start_async(self.lpr_cur_request_id, inputs={'seq_ind':seq_ind, 'data':in_frame})
                if self.lpr_previous_request_id >= 0:
                    status = self.lpr_infer_requests[self.lpr_previous_request_id].wait()
                    if status is not 0:
                        raise Exception("Infer request not completed successfully")
                    output_ = self.lpr_exec_net.requests[self.lpr_previous_request_id].outputs
                    onedarray = output_['decode'].reshape((88))
                    #print(type(onedarray))
                    text = ''
                    
                    for label_id in onedarray:
                        if(label_id == -1):
                            break
                        text = text + lpr_labels[int(label_id)]
                    
                    #lpr_result_dic[(nmsed_between_layer_results.index(obj))] = text
                    obj.lprtext = text
                    
                    
                self.lpr_cur_request_id += 1
                if self.lpr_cur_request_id >= self.num_requests:
                    self.lpr_cur_request_id = 0

                self.lpr_previous_request_id += 1
                if self.lpr_previous_request_id >= self.num_requests:
                    self.lpr_previous_request_id = 0
        #print('----------------------------------------get lpr end-----------------------------------')
        #print(lpr_result_dic)
        return nmsed_between_layer_results        
        
    def Detect(self, frame):
        print("NCS Detect")
        # ----------------------------------------------- 6. Doing inference
        # warming up - out of scope

        # Here is the first asynchronous point: in the Async mode, we capture frame to populate the NEXT infer request
        # in the regular mode, we capture frame to the CURRENT infer request

        in_frame_ = cv2.resize(frame, (self.w, self.h))
        in_frame = in_frame_[:,:,(2,1,0)] # BGR 2 RGB
        # resize input_frame to network size
        #in_frame = resize(frame, (self.w, self.h), preserve_range=True)
        in_frame = in_frame.transpose((2, 0, 1))  # Change data layout from HWC to CHW
        
        in_frame = in_frame.reshape((self.n, self.c, self.h, self.w))
        
        # Start inference
        #print('previous_request_id:{0};cur_request_id:{1}'.format(self.previous_request_id, self.cur_request_id))
        self.exec_net.start_async(self.cur_request_id, inputs={self.input_blob: in_frame})
        if self.previous_request_id >= 0:
            status = self.infer_requests[self.previous_request_id].wait(-1)
            if status is not 0:
                raise Exception("Infer request not completed successfully")
            output = self.exec_net.requests[self.previous_request_id].outputs
            if('TinyYoloV3' in self.base_detection):
                nmsed_between_layer_results = parse_ncs2_yolov3_tiny_output(output=output, frame=frame, detector=self.detector, W_H=self.w)
            else:
                nmsed_between_layer_results = self.parse_ncs2_ssd_output(output=output, frame=frame)
                # -------------------------------------------------- run LPR ---------------------------------------------------
            nmsed_between_layer_results = self.get_LPR(nmsed_between_layer_results, frame)
        self.cur_request_id += 1
        if self.cur_request_id >= self.num_requests:
            self.cur_request_id = 0

        self.previous_request_id += 1
        if self.previous_request_id >= self.num_requests:
            self.previous_request_id = 0

        return nmsed_between_layer_results

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

        imgb[offy:offy+newh,offx:offx+neww,:] = resize(img.copy()/255.0,(newh,neww),1)
        im = imgb[:,:,(2,1,0)]
        

        return im, int(offx*imgw/neww), int(offy*imgh/newh), neww/dim[0], newh/dim[1]
        #return transposed_img, int(offx*imgw/neww), int(offy*imgh/newh), neww/dim[0], newh/dim[1]

    def Reshape(self, out, dim):
        shape = out.shape
        out = np.transpose(out.reshape(self.wh, int(shape[0]/self.wh)))  
        out = out.reshape(shape)
        return out

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
