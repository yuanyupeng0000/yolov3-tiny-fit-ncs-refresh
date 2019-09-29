#ifndef __G_DEFINE__
#define __G_DEFINE__

#define uint8  unsigned char
#define uint16 unsigned short
#define uint32 unsigned int
//
#define VERTION_TIMEOUT 0
//
//相机方向代号
#define Z_NONE 0x0 
#define Z_NORTH 0x1 //北进口
#define Z_NORTHEAST 0x2 //东北进口
#define Z_EAST 0x4//  东进口
#define Z_SOUTHEAST 0x8//东南进口
#define Z_SOUTH 0x10//南进口
#define Z_SOUTHWEST 0x20//西南进口
#define Z_WEST 0x40//西进口
#define Z_NORTHWEST 0x80//西南进口
//
#define NANJING_LANE_COIL_MAX 2
#define PERSON_AREAS_MAX 8
#define PERSON_AREAS_TIMES_MAX 48

#define CLIENT_MAX 4
#define IPADDRLEN 16
#define SIGNALPORTL   4000
#define COMMONPORT 5000
#define MAX_LANE_NUM 8
#define MAX_LANE_BUS_NUM 50 //同一道上的车牌数
#define MAX_BUS_DATA 16
#define IP_LEN 16
#define CLIENT_MAX 16
#define CAM_NUM_1 8
#define ACTIVE_CAM_NUM 1
#define CLIENT_ACCESSMAX 8
#define TCP_SERVER_PORT 8888
#define SERVERPORT     12345
#define TIME_LINE_NUM 4
#define UDP_LOCAL_PORT 7000
#define USERNAMEMAX   20
#define COILPOINTMAX       4
#define DETECT_LANE_MAX_NUM   4
#define LANE_LINE_MAX         8
/////////////////////////////////
#define  STANDARD_VAULE_MAX    4
#define  ALG_MAX               2
#define  MAXSUBSYSTEM
#define  SECTIONMAX    50
#define  KEYMAX         50
#define  VALUEMAX       20
#define  DEVNAMEMAX    50
#define  TCP_TIMEOUT_SEC 20
#define  TIME_LEFT 60
#define  FILE_NAME_LENGTH 20
#define  FRAME_COLS  640
#define  FRAME_ROWS  480
#define  SDK_TYPE  1
#define  RTSP_TYPE 2
#define  CAMERA_OPEN_TYPE  RTSP_TYPE  //1--sdk  2--rtsp
//
#define  CAM_MAX 50
//
#define  FILENAME_SIZE 100
//
#define  DEF_BUS_NUMBER 1  //车牌
//
#define  TEST_PERSON_AREA_SIZE 6 //行人区域大小

enum enum_real_test_type{
    EM_CAR_OUT     = 0x01,
    EM_ONE_MINUTE  = 0x02,
    EM_FIVE_MINUTE = 0x04
};

enum enum_real_test_run_flag{
    EM_NOT     = 0x0,
    EM_RUNNING = 0x01,
    EM_RESET   = 0x02
};

#endif
