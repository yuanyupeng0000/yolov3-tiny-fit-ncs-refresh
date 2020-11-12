#!/usr/bin/env python
"""
 Copyright (c) 2018 Intel Corporation

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
"""
from __future__ import print_function, division

import logging
import os
import sys
from argparse import ArgumentParser
from math import exp as exp
from time import time

import cv2
from openvino.inference_engine import IENetwork, IEPlugin

import numpy as np
from libpydetector import YoloDetector
from Visualize import *

logging.basicConfig(format="[ %(levelname)s ] %(message)s", level=logging.INFO, stream=sys.stdout)
log = logging.getLogger()


def build_argparser():
    parser = ArgumentParser()
    parser.add_argument("-m", "--model", help="Path to an .xml file with a trained model.", required=True, type=str)
    parser.add_argument("-i", "--input", help="Path to a image/video file. (Specify 'cam' to work with camera)",
                        required=True, type=str)
    parser.add_argument("-l", "--cpu_extension",
                        help="MKLDNN (CPU)-targeted custom layers.Absolute path to a shared library with the kernels "
                             "impl.", type=str, default=None)
    parser.add_argument("-pp", "--plugin_dir", help="Path to a plugin folder", type=str, default=None)
    parser.add_argument("-d", "--device",
                        help="Specify the target device to infer on; CPU, GPU, FPGA or MYRIAD is acceptable. Sample "
                             "will look for a suitable plugin for device specified (CPU by default)", default="CPU",
                        type=str)
    parser.add_argument("--labels", help="Labels mapping file", default=None, type=str)
    parser.add_argument("-pt", "--prob_threshold", help="Probability threshold for detections filtering",
                        default=0.5, type=float)
    parser.add_argument("-iout", "--iou_threshold", help="Intersection over union threshold for overlapping detections"
                                                         " filtering", default=0.4, type=float)
    parser.add_argument("-ni", "--number_iter", help="Number of inference iterations", default=1, type=int)
    parser.add_argument("-pc", "--perf_counts", help="Report performance counters", default=False, action="store_true")
    return parser


class YoloV3Params:
    # ------------------------------------------- Extracting layer parameters ------------------------------------------
    # Magic numbers are copied from yolo samples
    def __init__(self, param, side):
        self.num = 3 if 'num' not in param else len(param['mask'].split(',')) if 'mask' in param else int(param['num'])
        self.coords = 4 if 'coords' not in param else int(param['coords'])
        #self.classes = 80 if 'classes' not in param else int(param['classes'])
        self.classes = 6 if 'classes' not in param else int(param['classes'])
        #self.anchors = [10.0, 13.0, 16.0, 30.0, 33.0, 23.0, 30.0, 61.0, 62.0, 45.0, 59.0, 119.0, 116.0, 90.0, 156.0, 198.0,
                        #373.0, 326.0] if 'anchors' not in param else [float(a) for a in param['anchors'].split(',')]
        self.anchors = [10,25,  20,50,  30,75, 50,125,  80,200,  150,150] if 'anchors' not in param else [float(a) for a in param['anchors'].split(',')]
        self.side = side
        if self.side == 13:
            self.anchor_offset = 2 * 3
        elif self.side == 26:
            self.anchor_offset = 2 * 0
        elif self.side == 52:
            self.anchor_offset = 2 * 0
        else:
            assert False, "Invalid output size. Only 13, 26 and 52 sizes are supported for output spatial dimensions"

    def log_params(self):
        params_to_print = {'classes': self.classes, 'num': self.num, 'coords': self.coords, 'anchors': self.anchors}
        [log.info("         {:8}: {}".format(param_name, param)) for param_name, param in params_to_print.items()]


def entry_index(side, coord, classes, location, entry):
    side_power_2 = side ** 2
    n = location // side_power_2
    loc = location % side_power_2
    return int(side_power_2 * (n * (coord + classes + 1) + entry) + loc)


def scale_bbox(x, y, h, w, class_id, confidence, h_scale, w_scale):
    xmin = int((x - w / 2) * w_scale)
    ymin = int((y - h / 2) * h_scale)
    xmax = int(xmin + w * w_scale)
    ymax = int(ymin + h * h_scale)
    return dict(xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax, class_id=class_id, confidence=confidence)


def parse_yolo_region(blob, resized_image_shape, original_im_shape, params, threshold):
    # ------------------------------------------ Validating output parameters ------------------------------------------
    _, _, out_blob_h, out_blob_w = blob.shape
    assert out_blob_w == out_blob_h, "Invalid size of output blob. It sould be in NCHW layout and height should " \
                                     "be equal to width. Current height = {}, current width = {}" \
                                     "".format(out_blob_h, out_blob_w)

    # ------------------------------------------ Extracting layer parameters -------------------------------------------
    orig_im_h, orig_im_w = original_im_shape
    resized_image_h, resized_image_w = resized_image_shape
    objects = list()
    predictions = blob.flatten()
    side_square = params.side * params.side

    # ------------------------------------------- Parsing YOLO Region output -------------------------------------------
    for i in range(side_square):
        row = i // params.side
        col = i % params.side
        for n in range(params.num):
            obj_index = entry_index(params.side, params.coords, params.classes, n * side_square + i, params.coords)
            scale = predictions[obj_index]
            if scale < threshold:
                continue
            box_index = entry_index(params.side, params.coords, params.classes, n * side_square + i, 0)
            x = (col + predictions[box_index + 0 * side_square]) / params.side * resized_image_w
            y = (row + predictions[box_index + 1 * side_square]) / params.side * resized_image_h
            # Value for exp is very big number in some cases so following construction is using here
            try:
                w_exp = exp(predictions[box_index + 2 * side_square])
                h_exp = exp(predictions[box_index + 3 * side_square])
            except OverflowError:
                continue
            w = w_exp * params.anchors[params.anchor_offset + 2 * n]
            h = h_exp * params.anchors[params.anchor_offset + 2 * n + 1]
            for j in range(params.classes):
                class_index = entry_index(params.side, params.coords, params.classes, n * side_square + i,
                                          params.coords + 1 + j)
                confidence = scale * predictions[class_index]
                if confidence < threshold:
                    continue
                objects.append(scale_bbox(x=x, y=y, h=h, w=w, class_id=j, confidence=confidence,
                                          h_scale=orig_im_h / resized_image_h, w_scale=orig_im_w / resized_image_w))
    return objects


def intersection_over_union(box_1, box_2):
    width_of_overlap_area = min(box_1['xmax'], box_2['xmax']) - max(box_1['xmin'], box_2['xmin'])
    height_of_overlap_area = min(box_1['ymax'], box_2['ymax']) - max(box_1['ymin'], box_2['ymin'])
    if width_of_overlap_area < 0 or height_of_overlap_area < 0:
        area_of_overlap = 0
    else:
        area_of_overlap = width_of_overlap_area * height_of_overlap_area
    box_1_area = (box_1['ymax'] - box_1['ymin']) * (box_1['xmax'] - box_1['xmin'])
    box_2_area = (box_2['ymax'] - box_2['ymin']) * (box_2['xmax'] - box_2['xmin'])
    area_of_union = box_1_area + box_2_area - area_of_overlap
    if area_of_union == 0:
        return 0
    return area_of_overlap / area_of_union

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

        imgb[offy:offy+newh,offx:offx+neww,:] = cv2.resize(img/255.0,(neww,newh))
        im = imgb[:,:,(2,1,0)]
        return im, int(offx*imgw/neww), int(offy*imgh/newh), neww/dim[0], newh/dim[1]
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

        imgb[offy:offy+newh,offx:offx+neww,:] = cv2.resize(img/255.0,(newh,neww))
        im = imgb[:,:,(2,1,0)]
        return im, int(offx*imgw/neww), int(offy*imgh/newh), neww/dim[0], newh/dim[1]
def non_max_suppress(predicts_dict, threshold=0.5):
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
def main():
    args = build_argparser().parse_args()

    model_xml = args.model
    model_bin = os.path.splitext(model_xml)[0] + ".bin"

    # ------------- 1. Plugin initialization for specified device and load extensions library if specified -------------
    plugin = IEPlugin(device=args.device, plugin_dirs=args.plugin_dir)
    if args.cpu_extension and 'CPU' in args.device:
        plugin.add_cpu_extension(args.cpu_extension)

    # -------------------- 2. Reading the IR generated by the Model Optimizer (.xml and .bin files) --------------------
    log.info("Loading network files:\n\t{}\n\t{}".format(model_xml, model_bin))
    net = IENetwork(model=model_xml, weights=model_bin)

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

    assert len(net.inputs.keys()) == 1, "Sample supports only YOLO V3 based single input topologies"
    #assert len(net.outputs) == 3, "Sample supports only YOLO V3 based triple output topologies"

    # ---------------------------------------------- 4. Preparing inputs -----------------------------------------------
    log.info("Preparing inputs")
    input_blob = next(iter(net.inputs))

    #  Defaulf batch_size is 1
    net.batch_size = 1

    # Read and pre-process input images
    n, c, h, w = net.inputs[input_blob].shape

    if args.labels:
        with open(args.labels, 'r') as f:
            labels_map = [x.strip() for x in f]
    else:
        labels_map = None

    input_stream = 0 if args.input == "cam" else args.input

    is_async_mode = True
    cap = cv2.VideoCapture(input_stream)
    number_input_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # Number of frames in picture is 1 and this will be read in cycle. Sync mode is default value for this case
    if number_input_frames != 1:
        ret, frame = cap.read()
    else:
        is_async_mode = False

    # ----------------------------------------- 5. Loading model to the plugin -----------------------------------------
    log.info("Loading model to the plugin")
    exec_net = plugin.load(network=net, num_requests=2)

    cur_request_id = 0
    next_request_id = 1
    render_time = 0
    parsing_time = 0

    # ----------------------------------------------- 6. Doing inference -----------------------------------------------
    detector = YoloDetector(1)
    while cap.isOpened():
        # Here is the first asynchronous point: in the Async mode, we capture frame to populate the NEXT infer request
        # in the regular mode, we capture frame to the CURRENT infer request
        if is_async_mode:
            ret, next_frame = cap.read()
        else:
            ret, frame = cap.read()

        if not ret:
            break

        if is_async_mode:
            request_id = next_request_id
            in_frame = cv2.resize(next_frame, (w, h))
        else:
            request_id = cur_request_id
            in_frame = cv2.resize(frame, (w, h))

        # resize input_frame to network size
        in_frame = in_frame.transpose((2, 0, 1))  # Change data layout from HWC to CHW
        #in_frame = in_frame/255.0
        #print(in_frame)
        in_frame = in_frame.reshape((n, c, h, w))

        # Start inference
        start_time = time()
        exec_net.start_async(request_id=request_id, inputs={input_blob: in_frame})
        det_time = time() - start_time

        # Collecting object detection results
        objects = list()
        if exec_net.requests[cur_request_id].wait(-1) == 0:
            output = exec_net.requests[cur_request_id].outputs

            start_time = time()
            for layer_name, out_blob in output.items():
                #print(layer_name)
                #print(out_blob)
                blockwd = 13
                classes = 6
                imgw = frame.shape[1]
                imgh = frame.shape[0]
                threshold = 0.5
                nms = 0.5
                targetBlockwd = 13
                ####################################################################################################
                if(layer_name == 'layer16-conv'):
                    out1 = out_blob.reshape(blockwd*blockwd*33)
                    internalresults1 = detector.Detect(out1.astype(np.float32), 33, blockwd, blockwd, classes, imgw, imgh, threshold, nms, targetBlockwd)
                    pyresults1 = [BBox(x,1,1, 0, 0) for x in internalresults1]
                elif(layer_name == 'layer23-conv'):
                    out2 = out_blob.reshape(blockwd*2*blockwd*2*33)
                    internalresults2 = detector.Detect(out2.astype(np.float32), 33, blockwd*2, blockwd*2, classes, imgw, imgh, threshold, nms, blockwd*2)
                    pyresults2 = [BBox(x,1,1, 0, 0) for x in internalresults2]
            pyresults3 = pyresults1 + pyresults2
            print(pyresults3)
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
            nms_pred_dict = non_max_suppress(pre_dic)
            if(nms_pred_dict == None):
                return []
            nmsed_between_layer_results = []
            for object_id, bboxes in nms_pred_dict.items():
                for bbox in bboxes:
                    bbox.append(object_id)
                    BBox__ = BBox_(bbox, 1, 1, 0, 0)
                    nmsed_between_layer_results.append(BBox__)

            #return nmsed_between_layer_results
            
            imdraw = Visualize(frame, nmsed_between_layer_results)
            if(imdraw.shape[0] >= 900 or imdraw.shape[1] >= 1440):
                imdraw = cv2.resize(imdraw, (int(imdraw.shape[1]/2), int(imdraw.shape[0]/2)))
            #fpsImg = cv2.putText(imdraw, "%.2ffps" % fps, (70, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1)
            fpsImg = imdraw
            cv2.imshow('Demo',fpsImg)
            #if cv2.waitKey(1) & 0xFF == ord('q'):
                #break

            ####################################################################################################
            '''layer_params = YoloV3Params(net.layers[layer_name].params, out_blob.shape[2])
            log.info("Layer {} parameters: ".format(layer_name))
            layer_params.log_params()
                
                
            objects += parse_yolo_region(out_blob, in_frame.shape[2:],
                                         frame.shape[:-1], layer_params,
                                         args.prob_threshold)
            parsing_time = time() - start_time

        # Filtering overlapping boxes with respect to the --iou_threshold CLI parameter
        for i in range(len(objects)):
            if objects[i]['confidence'] == 0:
                continue
            for j in range(i + 1, len(objects)):
                if intersection_over_union(objects[i], objects[j]) > args.iou_threshold:
                    objects[j]['confidence'] = 0

        # Drawing objects with respect to the --prob_threshold CLI parameter
        objects = [obj for obj in objects if obj['confidence'] >= args.prob_threshold]

        if len(objects):
            log.info("\nDetected boxes for batch {}:".format(1))
            log.info(" Class ID | Confidence | XMIN | YMIN | XMAX | YMAX | COLOR ")

        origin_im_size = frame.shape[:-1]
        for obj in objects:
            # Validation bbox of detected object
            if obj['xmax'] > origin_im_size[0] or obj['ymax'] > origin_im_size[0] or obj['xmin'] < 0 or obj['ymin'] < 0:
                continue
            color = (int(min(obj['class_id'] * 12.5, 255)),
                     min(obj['class_id'] * 7, 255), min(obj['class_id'] * 5, 255))
            det_label = labels_map[obj['class_id']] if labels_map and len(labels_map) >= obj['class_id'] else \
                str(obj['class_id'])
            log.info("{:^9} | {:10f} | {:4} | {:4} | {:4} | {:4} | {} ".format(det_label, obj['confidence'], obj['xmin'],
                                                                               obj['ymin'], obj['xmax'], obj['ymax'],
                                                                               color))

            cv2.rectangle(frame, (obj['xmin'], obj['ymin']), (obj['xmax'], obj['ymax']), color, 2)
            cv2.putText(frame,
                        "#" + det_label + ' ' + str(round(obj['confidence'] * 100, 1)) + ' %',
                        (obj['xmin'], obj['ymin'] - 7), cv2.FONT_HERSHEY_COMPLEX, 0.6, color, 1)

        # Draw performance stats over frame
        inf_time_message = "Inference time: N\A for async mode" if is_async_mode else \
            "Inference time: {:.3f} ms".format(det_time * 1e3)
        render_time_message = "OpenCV rendering time: {:.3f} ms".format(render_time * 1e3)
        async_mode_message = "Async mode is on. Processing request {}".format(cur_request_id) if is_async_mode else \
            "Async mode is off. Processing request {}".format(cur_request_id)
        parsing_message = "YOLO parsing time is {:.3f}".format(parsing_time * 1e3)

        cv2.putText(frame, inf_time_message, (15, 15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (200, 10, 10), 1)
        cv2.putText(frame, render_time_message, (15, 45), cv2.FONT_HERSHEY_COMPLEX, 0.5, (10, 10, 200), 1)
        cv2.putText(frame, async_mode_message, (10, int(origin_im_size[0] - 20)), cv2.FONT_HERSHEY_COMPLEX, 0.5,
                    (10, 10, 200), 1)
        cv2.putText(frame, parsing_message, (15, 30), cv2.FONT_HERSHEY_COMPLEX, 0.5, (10, 10, 200), 1)

        start_time = time()
        cv2.imshow("DetectionResults", frame)
        render_time = time() - start_time'''

        if is_async_mode:
            #print('is_async_mode')
            cur_request_id, next_request_id = next_request_id, cur_request_id
            frame = next_frame

        key = cv2.waitKey(1)
        # Tab key
        if key == 27:
            break
        # ESC key
        if key == 9:
            exec_net.requests[cur_request_id].wait()
            is_async_mode = not is_async_mode
            log.info("Switched to {} mode".format("async" if is_async_mode else "sync"))

    cv2.destroyAllWindows()

    del net
    del exec_net
    del plugin


if __name__ == '__main__':
    sys.exit(main() or 0)
