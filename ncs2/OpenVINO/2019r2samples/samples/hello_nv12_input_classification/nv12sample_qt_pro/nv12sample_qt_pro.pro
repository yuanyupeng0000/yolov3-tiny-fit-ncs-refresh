SUBDIRS += \
    nv12sample_qt_pro.pro

DISTFILES += \
    ../README.md \
    ../CMakeLists.txt

SOURCES += \
    ../main.cpp

INCLUDEPATH += /opt/intel/openvino/deployment_tools/inference_engine/include \
    /opt/intel/openvino/deployment_tools/inference_engine/samples/common \
    /opt/intel/openvino/deployment_tools/inference_engine/src/extension \
    ../
