DISTFILES += \
    ../README.md \
    ../CMakeLists.txt

HEADERS += \
    ../test_lib_app/intel_dldt.h \
    ../security_barrier_camera.hpp

SOURCES += \
    ../test_lib_app/test.cpp \
    ../intel_dldt.cpp

INCLUDEPATH += /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/include \
    /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/common \
    /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/src/extension
