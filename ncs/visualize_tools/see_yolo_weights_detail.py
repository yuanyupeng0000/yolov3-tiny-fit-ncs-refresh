import numpy as np  
from collections import OrderedDict

from ConfigParser import ConfigParser

cfgfile = '/data/darknet/cfg/yolov3-tiny.cfg'
weights = '/data/darknet/weights/yolov3-tiny.weights'

class uniqdict(OrderedDict):
    _unique = 0
    def __setitem__(self, key, val):
        if isinstance(val, OrderedDict):
            self._unique += 1
            key += "_"+str(self._unique)
        OrderedDict.__setitem__(self, key, val)
parser = ConfigParser(dict_type=uniqdict)
parser.read(cfgfile)
blocks = parser.sections()
print(blocks)
fp = open(weights, "rb")
help(fp.read)
#### The first 4 values are header information   
# 1. Major version number  
# 2. Minor Version Number  
# 3. Subversion number   
# 4. IMages seen   
header = np.fromfile(fp, dtype = np.int32, count = 5)  
print(header)
#fp = open(weightfile, 'rb')  
#header = np.fromfile(fp, count=5, dtype=np.int32)  
#header = np.ndarray(shape=(5,),dtype='int32',buffer=fp.read(20))  
#print(header)  
buf = np.fromfile(fp, dtype = np.float32)
help(np.fromfile)
#print(buf)
start = 0
for block in blocks:
        print(block) 
        items = dict(parser.items(block))
        print(items)
        if(block.startswith('convolutional')):
            filters = int(items.get('filters'))
            size = int(items.get('size'))
            bn = items.get('batch_normalize')
            if(bn == '1'):             
                step = size**2*filters + filters*4                
                print(buf[start:step])
                start += step
                print(start)
                print(len(buf))
