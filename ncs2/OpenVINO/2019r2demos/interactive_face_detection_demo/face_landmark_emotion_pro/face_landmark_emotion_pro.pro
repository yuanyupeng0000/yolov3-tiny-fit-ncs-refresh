SUBDIRS += \
    face_landmark_emotion_pro.pro

DISTFILES += \
    ../README.md \
    ../models.lst \
    ../CMakeLists.txt

HEADERS += \
    ../detectors.hpp \
    ../face.hpp \
    ../interactive_face_detection.hpp \
    ../visualizer.hpp

SOURCES += \
    ../detectors.cpp \
    ../face.cpp \
    ../main.cpp \
    ../visualizer.cpp

INCLUDEPATH += /opt/intel/openvino/deployment_tools/inference_engine/include \
    /opt/intel/openvino/deployment_tools/inference_engine/samples/common \
    /opt/intel/openvino/deployment_tools/inference_engine/src/extension \
    ../
