import xml.etree.ElementTree as ET
import cv2
def get_image_shape(image_full_name):
    image = cv2.imread(image_full_name)
    return image.shape
def generate_xml_str_head(image_full_name, generated_xml_dir):
    (h,w,c) = get_image_shape(image_full_name)
    xml_str = "<annotation>\n\t\
    <folder>JPEGImages</folder>\n\t\
    <filename>" + image_full_name.split('/')[-1] + "</filename>\n\t\
    " + "<path>" + image_full_name + "</path>\n\t\
    <source>\n\t\t\
    <database>ZENITH</database>\n\t\
    </source>\n\t\
    <size>\n\t\t\
    <width>" + str(w) + "</width>\n\t\t\
    <height>" + str(h) + "</height>\n\t\t\
    <depth>" + str(c) + "</depth>\n\t\
    </size>\n\t\
    <segmented>0</segmented>"
    return xml_str

def generate_obj_xml_str(rect, cls_name):
    xmin = int(rect[0])
    ymin = int(rect[1])
    xmax = int(xmin + rect[2])
    ymax = int(ymin + rect[3])
    obj_str = "\n\t\
    <object>\n\t\t\
    <name>" + cls_name + "</name>\n\t\t\
    <pose>Unspecified</pose>\n\t\t\
    <truncated>0</truncated>\n\t\t\
    <difficult>0</difficult>\n\t\t\
    <bndbox>\n\t\t\t\
    <xmin>" + str(int(xmin)) + "</xmin>\n\t\t\t\
    <ymin>" + str(int(ymin)) + "</ymin>\n\t\t\t\
    <xmax>" + str(int(xmax)) + "</xmax>\n\t\t\t\
    <ymax>" + str(int(ymax)) + "</ymax>\n\t\t\
    </bndbox>\n\t\
    </object>"
    return obj_str   
    
def append_xml_obj_str(xml_str, obj_xml_str):
    xml_str += obj_xml_str
    return xml_str

def append_xml_str_tail(xml_str):
    xml_str += "\n</annotation>"
    return xml_str

def write_xml_str_to_file(xml_str, full_name):
    fileObject = open(full_name, 'w')  
    fileObject.write(xml_str)
    fileObject.close()



### std_obj: left top right bottom confidence objType name ###
def generate_xml(std_objs_per_frame, image_full_name, generated_xml_dir):
    xml_str = generate_xml_str_head(image_full_name, generated_xml_dir)
    image_name = image_full_name.split('/')[-1]
    for obj in std_objs_per_frame:
        rect = [obj.left, obj.top, obj.right-obj.left, obj.bottom-obj.top]
        cls_name = obj.name
        confidence = obj.confidence
        print('cls_name:{0},confidence:{1}'.format(cls_name, confidence))
        if(cls_name == 'plate' and confidence > 0.1):
            obj_xml_str = generate_obj_xml_str(rect, cls_name)
            xml_str = append_xml_obj_str(xml_str, obj_xml_str)
    xml_str = append_xml_str_tail(xml_str)
    full_name_xml = generated_xml_dir + '/' + image_name[:-3] + 'xml'
    write_xml_str_to_file(xml_str, full_name_xml)



