#!/bin/bash

filename=yolov3-tiny-ncs
yolocfg=./$filename.cfg
yoloweight=./$filename.weights


yolocfgcaffe=./$filename.prototxt
yoloweightcaffe=./$filename.caffemodel

echo $yolocfg
echo $yoloweight

echo "convert yolo to caffe"
python ./create_yolo_prototxt.py $yolocfg $yolocfgcaffe
python ./create_yolo_caffemodel.py -m $yolocfgcaffe -w $yoloweight -o $yoloweightcaffe

echo "done"
