# Pytorch darknet to caffe 

Modify from [pytorch-caffe-darknet-convert](https://github.com/marvis/pytorch-caffe-darknet-convert),[object_detetction_tools](https://github.com/BingzheWu/object_detetction_tools)

## Modified items :

1. yolov3 output layer
2. when pooling layer stide =1 , size =2 , assign size = 1
3. upsample layer 

## Usage : 

1. Download weights from original darknet web
2. Unmark custom_class in examples\ssd\ssd_detect.cpp
3. Remake project
 
```
> python darknet2caffe.py yolov3.cfg yolov3.weights yolov3.prototxt yolov3.caffemodel
> cd $caffe_root
> sh demo_darknet_yolov3.sh
```


## Retrain  :

You can try  retrain models to approach original darknet  mAP , below was my test

Network|mAP|Resolution|iters
:---:|:---:|:---:|:---:
yolov3-spp|58.7|608|100
yolov3-spp|59.0|608|200
yolov3-spp|59.8|608|1000
