import os, sys, datetime 
if(len(sys.argv) < 2):
    print('please input : yolov3-tiny-ncs-weights file')
    exit(1)
weights = sys.argv[1]
#cfg = sys.argv[2]

#sys.path.insert(0, '/data/ssd-caffe/py2_caffe/python') 
sys.path.insert(0, '/data/ssd-caffe/new-yolov3-caffe/python')
import caffe  
import numpy as np  
from collections import OrderedDict
from ConfigParser import ConfigParser


#from ConfigParser import ConfigParser
class uniqdict(OrderedDict):
    _unique = 0
    def __setitem__(self, key, val):
        if isinstance(val, OrderedDict):
            self._unique += 1
            key += "_"+str(self._unique)
        OrderedDict.__setitem__(self, key, val)
def load_conv2caffe(buf, start, conv_param):  
    weight = conv_param[0].data  
    bias = conv_param[1].data  
    conv_param[1].data[...] = np.reshape(buf[start:start+bias.size], bias.shape);   start = start + bias.size  
    conv_param[0].data[...] = np.reshape(buf[start:start+weight.size], weight.shape); start = start + weight.size  
    return start
def load_fc2caffe(buf, start, fc_param):  
    weight = fc_param[0].data  
    bias = fc_param[1].data  
    fc_param[1].data[...] = np.reshape(buf[start:start+bias.size], bias.shape);   start = start + bias.size  
    fc_param[0].data[...] = np.reshape(buf[start:start+weight.size], weight.shape); start = start + weight.size  
    return start
def load_conv_bn2caffe(buf, start, conv_param, bn_param, scale_param): 
    conv_weight = conv_param[0].data  
    running_mean = bn_param[0].data  
    running_var = bn_param[1].data  
    scale_weight = scale_param[0].data  
    scale_bias = scale_param[1].data      
    scale_param[1].data[...] = np.reshape(buf[start:start+scale_bias.size], scale_bias.shape); start = start + scale_bias.size  
    #print scale_bias.size  
    #print scale_bias  
  
    scale_param[0].data[...] = np.reshape(buf[start:start+scale_weight.size], scale_weight.shape); start = start + scale_weight.size  
    #print scale_weight.size
    #print scale_weight
  
    bn_param[0].data[...] = np.reshape(buf[start:start+running_mean.size], running_mean.shape); start = start + running_mean.size  
    #print running_mean.size
    #print running_mean
  
    bn_param[1].data[...] = np.reshape(buf[start:start+running_var.size], running_var.shape); start = start + running_var.size  
    #print running_var.size
    #print running_var
  
    bn_param[2].data[...] = np.array([1.0])  
    conv_param[0].data[...] = np.reshape(buf[start:start+conv_weight.size], conv_weight.shape); start = start + conv_weight.size  
    #print conv_weight.size
    #print conv_weight
  
    return start
def darknet2caffe(cfgfile, weightfile, protofile, caffemodel='gene.caffemodel'):  
    #net_info = cfg2prototxt(cfgfile)
    #save_prototxt(net_info , protofile, region=False)  
    print('benchmark')
    net = caffe.Net(protofile, caffe.TEST)
    k_v_s = [(k, v) for k, v in net.params.items()]
    key_vecnums = [(vecs[0], len(vecs[1])) for vecs in k_v_s]
    print([(vecs[0], [vecs[1][i].data.shape for i in range(len(vecs[1]))])for vecs in k_v_s])
    print('benchmark')
    params = net.params
    print('benchmark')
  
    #blocks = parse_cfg(cfgfile)
    parser = ConfigParser(dict_type=uniqdict)
    parser.read(cfgfile)
    blocks = parser.sections()
    print(blocks)
    
  
    #Open the weights file  
    fp = open(weightfile, "rb")  
  
    #The first 4 values are header information   
    # 1. Major version number  
    # 2. Minor Version Number  
    # 3. Subversion number   
    # 4. IMages seen   
    header = np.fromfile(fp, dtype = np.int32, count = 5)
    #header = np.fromfile(fp, dtype = np.float32, count = 5)
    print(header)
    #fp = open(weightfile, 'rb')  
    #header = np.fromfile(fp, count=5, dtype=np.int32)  
    #header = np.ndarray(shape=(5,),dtype='int32',buffer=fp.read(20))  
    #print(header)  
    buf = np.fromfile(fp, dtype = np.float32)
    print('buf len:{0}'.format(len(buf)))
    #print(buf)  
    fp.close()  
  
    layers = []  
    layer_id = 1  
    start = 0  
    for block in blocks:
        print(block)
        if start >= buf.size:  
            break
        items = dict(parser.items(block))
        print(items)
        if block.split('_')[0] == 'net':  
            continue
        elif ((block.split('_')[0] == 'convolutional') or 
        (block.split('_')[0] == 'deconvolutional')):
            batchnorm_followed = False
            relu_followed = False
            
            if 'batch_normalize' in items and items['batch_normalize']:
                batchnorm_followed = True
            if 'activation' in items and items['activation'] != 'linear':
                relu_followed = True
            
            if items.has_key('name'):  
                conv_layer_name = items['name']  
                print('has key name ' + conv_layer_name)
                bn_layer_name = '%s-bn' % items['name']  
                scale_layer_name = '%s-scale' % items['name']  
            else:
                if(block.split('_')[0] == 'deconvolutional'):
                    conv_layer_name = 'layer%d-upsample' % layer_id  
                    print('has no name ' + conv_layer_name)
                    #bn_layer_name = 'layer%d-bn' % layer_id  
                    #scale_layer_name = 'layer%d-scale' % layer_id 
                else:
                    conv_layer_name = 'layer%d-conv' % layer_id  
                    print('has no name ' + conv_layer_name)
                    bn_layer_name = 'layer%d-bn' % layer_id  
                    scale_layer_name = 'layer%d-scale' % layer_id  
  
            if batchnorm_followed:
                print("load_conv_bn2caffe:")
                start = load_conv_bn2caffe(buf, start, params[conv_layer_name], 
                                           params[bn_layer_name], params[scale_layer_name])
            else:
                print("load_conv2caffe:")
                start = load_conv2caffe(buf, start, params[conv_layer_name])  
            if(layer_id == 11):
                print('layer_id == 11')
                layer_id = layer_id + 2
            else:
                layer_id = layer_id+1
            print('start:{0}'.format(start))
        elif block.split('_')[0] == 'connected':  
            if items.has_key('name'):  
                fc_layer_name = items['name']  
            else:  
                fc_layer_name = 'layer%d-fc' % layer_id  
            start = load_fc2caffe(buf, start, params[fc_layer_name])  
            layer_id = layer_id+1  
        elif block.split('_')[0] == 'maxpool':  
            layer_id = layer_id+1  
        elif block.split('_')[0] == 'avgpool':  
            layer_id = layer_id+1  
        elif block.split('_')[0] == 'region':  
            layer_id = layer_id + 1  
        elif block.split('_')[0] == 'route':  
            layer_id = layer_id + 1  
        elif block.split('_')[0] == 'shortcut':  
            layer_id = layer_id + 1  
        elif block.split('_')[0] == 'softmax':  
            layer_id = layer_id + 1  
        elif block.split('_')[0] == 'cost':  
            layer_id = layer_id + 1  
        elif block.split('_')[0] == 'upsample':  
            layer_id = layer_id + 1 
        else:  
            print('unknow layer type %s ' % block.split('_')[0]) 
            layer_id = layer_id + 1 
    print('save caffemodel to %s' % caffemodel)  
    net.save(caffemodel)
    
cfgfile = '/data/github_repos/yolov3-tiny-fit-ncs/ncs/yolov3-tiny-ncs-without-last-maxpool.cfg'
#weights = weights
prototxt = '/data/github_repos/yolov3-tiny-fit-ncs/ncs/yolov3-tiny-ncs-without-last-maxpool.prototxt'
#caffemodel = 'Jenerated_nolastpooling.caffemodel'
caffemodel = (datetime.datetime.now()).strftime("%Y%m%d%H%M%S_") + 'TinyYoloV3NCS.caffemodel'
darknet2caffe(cfgfile, weights, prototxt, caffemodel)

mvnc_command = 'sh generate-graph.sh ' + caffemodel + ' ' + prototxt
os.system(mvnc_command)
set_ncs2_env_command = 'sh source /opt/intel/computer_vision_sdk/bin/setupvars.sh'
os.system(set_ncs2_env_command)
make_prototxt_command = 'cp /data/github_repos/yolov3-tiny-fit-ncs/ncs/convert_tools/TinyYoloV3NCS.prototxt ' + caffemodel[:-10] + 'prototxt'
os.system(make_prototxt_command)
mo_command = 'sh /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/model_optimizer/yolov3-tiny-mo.sh ' + '/data/github_repos/yolov3-tiny-fit-ncs/ncs/convert_tools/' + caffemodel
os.system(mo_command)

