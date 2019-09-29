import logging
import sys, os
from math import exp as exp
import caffe
import matplotlib.pyplot as plt
from argparse import ArgumentParser
logging.basicConfig(format="[ %(levelname)s ] %(message)s", level=logging.INFO, stream=sys.stdout)
log = logging.getLogger()
caffe_root = '/data/ssd-caffe/new-yolov3-caffe'
img_file = '/data/github_repos/yolov3-tiny-fit-ncs/ncs/standard_size_416_persons.jpg'
prototxt = '/data/NCS/OpenVINO/tensorflow-yolo-v3/model_optimizer/20190218182235_TinyYoloV3NCS.prototxt'
caffemodel = '/data/NCS/OpenVINO/tensorflow-yolo-v3/model_optimizer/20190218182235_TinyYoloV3NCS.caffemodel'
sys.path.insert(0, caffe_root + '/python')

#%matplotlib inline
caffe.set_mode_cpu()
net = caffe.Net(prototxt, caffemodel, caffe.TEST)

#img_caffe_io = caffe.io.load_image(img_file)
#print(img.shape)
#plt.imshow(img)
import cv2
import numpy as np
cv2.__version__

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
    print(original_im_shape)
    resized_image_h, resized_image_w = resized_image_shape
    print(resized_image_shape)
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

def build_argparser():
    parser = ArgumentParser()
    parser.add_argument("-pt", "--prob_threshold", help="Probability threshold for detections filtering",
                        default=0.5, type=float)
    parser.add_argument("-iout", "--iou_threshold", help="Intersection over union threshold for overlapping detections"
                                                         " filtering", default=0.4, type=float)
    parser.add_argument("-ni", "--number_iter", help="Number of inference iterations", default=1, type=int)
    parser.add_argument("-pc", "--perf_counts", help="Report performance counters", default=False, action="store_true")
    parser.add_argument("-i", "--input", help="input_video", default='/video/09_58_09.264', type=str)
    return parser


#objects = list()
args = build_argparser().parse_args()
video_file = args.input

cap = cv2.VideoCapture(video_file)
while cap.isOpened():
    ret, img = cap.read()
    if ret == True:
        objects = []
    #img = cv2.imread(img_file) ###BGR
        resized_img = cv2.resize(img, (416,416))
        #resized_img = cv2.cvtColor(resized_img, cv2.COLOR_BGR2RGB) # cv2默认为bgr顺序
        img_ = np.transpose(resized_img, (2, 0, 1))
        print(img_.shape)
        #print(img_)
        normerlized_img = img_/255.0
        #normerlized_img = img_
        #print(normerlized_img)

        net.blobs['data'].data[...] = normerlized_img
        inputData = net.blobs['data'].data[...]
        #inputData
        outputs = net.forward()
        #print(outputs)


        for layer_name, out_blob in outputs.items():
            #print(layer_name)
            #print(out_blob)
            #layer_params = YoloV3Params(net.layers[layer_name].params, out_blob.shape[2])
            layer_params = YoloV3Params('', out_blob.shape[2])

            log.info("Layer {} parameters: ".format(layer_name))
            layer_params.log_params()
            in_frame = inputData
            frame = img
            objects += parse_yolo_region(out_blob, in_frame.shape[2:],
                                     frame.shape[:-1], layer_params,
                                     args.prob_threshold)

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
            #det_label = labels_map[obj['class_id']] if labels_map and len(labels_map) >= obj['class_id'] else str(obj['class_id'])
            log.info("{:10f} | {:4} | {:4} | {:4} | {:4} | {} ".format(obj['confidence'], obj['xmin'],
                                                                           obj['ymin'], obj['xmax'], obj['ymax'],
                                                                           color))

            cv2.rectangle(img, (obj['xmin'], obj['ymin']), (obj['xmax'], obj['ymax']), color, 2)
            #cv2.putText(img,
                    #"#" + ' ' + str(round(obj['confidence'] * 100, 1)) + ' %',
                    #(obj['xmin'], obj['ymin'] - 7), cv2.FONT_HERSHEY_COMPLEX, 0.6, color, 1)

        # Draw performance stats over frame
            #cv2.putText(img, "test", (15, 30), cv2.FONT_HERSHEY_COMPLEX, 0.5, (10, 10, 200), 1)
        cv2.imshow("DetectionResults", img)
        key = cv2.waitKey(5)
        
        # Tab key
        if key == 27:
            break
        # ESC key
        if key == 9:
            break   
        
cv2.destroyAllWindows()
del net
