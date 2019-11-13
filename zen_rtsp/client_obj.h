/*
 * protocol.h
 *
 *  Created on: 2016��8��17��
 *      Author: root
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <pthread.h>
#include "g_define.h"
//
#define SET_BASE_CMD 0x1001
#define READ_BASE_CMD 0x1002
#define RETURN_BASE_CMD 0x1003

#define  VERSION       0x01        //�汾����
#define  PROTTYPE      0x02        //Э��汾

//����Э������
#define  GETSYSPARAM   0x0101
#define  REPSYSPARAM   0x0102
//������Ϣ
#define  GETBASEPARAM   0x1002      //���������Ϣ
#define  SETBASEPARAM   0x1001	   //���û�����Ϣ
#define  REPBASEPARAM   0x1003	   //���û�����Ϣ
//�������
#define  GETNETWORKPARAM   0x1008      //�������������Ϣ
#define  SETNETWORKPARAM   0x1007	   //�����������
#define  REPNETWORKPARAM   0x1009	   //�ظ����������Ϣ
//�л�ʱ������
#define  GETCHTIMEPARAM   0x1035
#define  SETCHTIMEPARAM   0x1034
#define  REPCHTIMEPARAM   0x1036
//��������
#define  GETSERIALPARAM   0x1029
#define  SETSERIALPARAM   0x1028
#define  REPSERIALPARAM   0x1030
//ͳ�Ʋ�������
#define  GETSTATISPARAM   0x2017
#define  SETSTATISPARAM   0x2016
#define  REPSTATISPARAM   0x2018
//�㷨����
#define  GETALGSPARAM   0x2014
#define  SETALGPARAM    0x2013
#define  REPALGPARAM   0x2015
//ʱ���������
#define  GETDATEPARAM   0x2030
#define  SETDATEPARAM   0x2032
#define  REPDATEPARAM   0x2031
//Э���������
//#define  GETPROTOCOLPARAM  0x1042
//#define  SETPROTOCOLPARAM  0x1041
//#define  REPPROTOCOLPARAM  0x1043
//ntp��������
#define  GETNTPPARAM       0x7001
#define  SETNTPPARAM       0x7002
#define  REPNTPPARAM       0x7003
//
#define  GETCAMERANUMPARAM    0x6001  //��ȡ�������
#define  REPCAMERANUMPARAM    0x6002 //�ظ�
#define  DELCAMERANUMPARAM    0x6004  //ɾ�����

#define SETPERSONARETIM 0x8001 //��������ʱ������
#define GETPERSONARETIM 0x8002 //��ȡ����ʱ������
#define REPPERSONARETIM 0x8003 //�ظ�����ʱ������

#define SETEVENTPARAM 0x9001 //�¼�����������
#define GETEVENTPARAM 0x9002 //�¼�����������
#define REPEVENTPARAM 0x9003 //�ظ��¼�����������
//�豸����
#define SETDEVREBOOT 0x5003

//
#define AREAMAX 8
//#########################################################
#define  SETDETECTDEVICE    0x0004    //���ü���豸��������
#define  GETDETECTDEVICE    0x0005    //��ȡ����豸��������
#define  REPDETECTDEVICE    0x0006    //��ȡ����豸����������Ӧ

#define  SETCHECHPARAM    0x0007    //������Ƶ���߲���
#define  GETCHECHPARAM    0x0008    //��ȡ��Ƶ���߲���
#define  REPCHECHPARAM    0x0009    //��ȡ��Ƶ���߲�����Ӧ

#define  HEART    0x1000    //����������Э��

#define  SHUTDOWN         0x1050    //�ر�����
#define  STARTREALDATA    0x1051    //��ʼ����
#define  REALTESTDATA     0x1053    //�������ݻ�Ӧ

#define  REALDATA    0x1060    //ʵʱ����

#define  REPHEART    0x1002    //����������Э���Ӧ
#define  REBOOTZEN   0x2001   //��������
#define  FORKEXIT    0x3001   //�����쳣�˳�

#define  HEARTTIME     60
#define  USERNAMEMAX   20
#define  IPADDRMAX     16
#define  DEVNAMEMAX    50
#define  FILEPATHMAX   100

//����������
#define Z_NONE 0x0
#define Z_NORTH 0x1
#define Z_NORTHEAST 0x2
#define Z_EAST 0x4
#define Z_SOUTHEAST 0x8
#define Z_SOUTH 0x10
#define Z_SOUTHWEST 0x20
#define Z_WEST 0x40
#define Z_NORTHWEST 0x80

#define  COILPOINTMAX        4      //��Ȧ�ĸ�����
#define  DETECTLANENUMMAX    8      //���ͨ������
#define  LANELINEMAX         2*DETECTLANENUMMAX  //�����߸���
#define  STANDPOINT          4      //�궨��8������
#define  STANDARDVAULEMAX    4
#define  ALGMAX              2      //�㷨��������
#define  MAXSUBSYSTEM        4
#define  DETECT_AREA_MAX     8

#define BOX_SIZE 200

#pragma pack(push)
#pragma pack(1)

typedef struct Command{
	unsigned char version;       //�汾����VERSION
	unsigned char prottype;      //Э��汾 PROTTYPE
	unsigned short objnumber;      //��������. �±�index
	unsigned short objtype;      //��������   ����: SETCAMPARAM
	unsigned int objlen;         //������ĳ���
}mCommand; //����Э��ͷ

typedef struct
{
	uint16 x;
	uint16 y;
	uint16 width;
	uint16 height;
	int label;
    int confidence;
	int id;
	int distance;
	int landid;
}IVDCRect;


typedef struct{
    unsigned short x;
    unsigned short y;
}IVDPoint;
/*-----------------ʵʱ�������-----------------*/
typedef struct{
    unsigned char   state;                    //����״̬   //0����  1�복
    unsigned char   isCarMid;        //�����ڶ���Ȧ״̬���м���Ȧ�� //0����  1�복
    unsigned char   isCarInTail;      //������һ��Ȧ״̬����Ƶ���¶���Ȧ�� 0����, 1��
    unsigned short  queueLength;              //���ӵĳ���
    //unsigned int    vehnum;                   //��������
    unsigned int    vehnum1;           //��һ��Ȧ��������
    unsigned int    vehnum2;           //�ڶ���Ȧ��������
    unsigned int    vehlength1;           //��һ��Ȧ��������
    unsigned int    vehlength2;           //�ڶ���Ȧ��������
    //unsigned int    speed; 					  //�������ٶ�
    unsigned int    speed1; 				//��һ��Ȧ�������ٶ�
    unsigned int    speed2; 				//�ڶ���Ȧ�������ٶ�
    unsigned int    existtime1;      //��һ��Ȧ����ʱ�� ��λms
    unsigned int    existtime2;      //�ڶ���Ȧ����ʱ�� ��λms
    unsigned short  uActualDetectLength;       //������Ȧ�ĳ���  //����Ȧ
    unsigned short  uActualTailLength;			//������Ȧ�ĳ���  //Զ��Ȧ
    IVDPoint        LineUp[2];                //��ǰ����ʵ������ ��ʼ����յ�
    unsigned int  BicycleFlow;//���г�����
	unsigned int  BusFlow;//����������
	unsigned int  CarFlow;//С������
	unsigned int  TruckFlow;//��������
	unsigned int  MotorbikeFlow;//Ħ�г�����
	unsigned int  nVehicleFlow; //�ǻ���������
	unsigned char LaneDirection;//��ⷽ��0��ʾ��������Ϊ���򣨼���ͷ����1��ʾȥ������Ϊ���򣨼���β��
	unsigned int  Headway;//��ͷ���
}mRealLaneInfo;
typedef struct{
    int x;
    int y;
    int w;
    int h;
    int label;
    int confidense;
	int id;
	int distance;
	int landid;
}fvd_objects;

typedef struct{
unsigned char id;//����id   
uint16 personNum;//������������
unsigned int upperson;// ������������
unsigned int downperson;//����������

}mRealPersonInfo;


typedef struct{
    unsigned char   flag;          //���ݱ�־0xFF
    unsigned int    deviceId;//�豸ID
	unsigned int    camId;//���ID
    unsigned char   laneNum;       //ʵ�ʳ�������
    unsigned char   curstatus;      //  1 �ǰ���, 2 ��ҹ��
    unsigned char   fuzzyflag;                //��Ƶ�쳣״̬
    unsigned char   visibility;		           //�ܼ���״̬
    unsigned short 	uDegreePoint[20][2];      //���������. 0:x 1:y
    //unsigned short 	uDegreePoint[4][2];      //���������. 0:x 1:y
    mRealLaneInfo   lane[DETECTLANENUMMAX];  //16
    // nanjing....
    unsigned char   area_car_num[DETECTLANENUMMAX];//car amount
    unsigned char   queue_len[DETECTLANENUMMAX];// len
    IVDPoint queue_line[DETECTLANENUMMAX][2];
    int rcs_num;
    fvd_objects rcs[BOX_SIZE];
    //IVDPoint detectline[2];//���˼�������꣬0��������1�յ�����

   // unsigned int upperson;// ������������
   // unsigned int downperson;//����������
    IVDCRect udetPersonBox[BOX_SIZE];//���˼���
	uint16   udetPersonNum;//���˼�����
    mRealPersonInfo personRegion[6];//����������
    unsigned int BicycleFlow1;//���г���������
	unsigned int BicycleFlow2;//���г���������
	unsigned int MotorbikeFlow1;//Ħ�г���������
	unsigned int MotorbikeFlow2;//Ħ�г���������
}mRealStaticInfo;

#define CAM_CLOSED_STATUS 0
#define CAM_OPENED_STATUS 1
typedef struct caminfo {

	unsigned char camstatus;
	unsigned char camdirect;
	unsigned char cammerIp[IPADDRMAX];
} m_caminfo;

typedef struct DetectDeviceConfig{
	unsigned int  deviceID;   //�����ID
	unsigned int  detectport;
	unsigned char camnum;
	unsigned char detectip[IPADDRMAX];
	unsigned char detectname[DEVNAMEMAX];
	m_caminfo cam_info[CAM_NUM_1];

}mDetectDeviceConfig;   //������豸����,����4��������,

//-----------------��������----���������ز�������-------------
typedef struct CamAttributes{
	unsigned char direction;
	unsigned int  camID;      //�����ID
	unsigned int  cammerport;
	unsigned int  adjustport;
	unsigned int  signalport;
	unsigned char urlname[USERNAMEMAX];  //Ϊ���ʵ����ļ�����
	unsigned char username[USERNAMEMAX];
	unsigned char passwd[USERNAMEMAX];
	unsigned char cammerIp[IPADDRMAX];
	unsigned char adjustIp[IPADDRMAX];
	unsigned char signalIp[IPADDRMAX];
}mCamAttributes; //���������

typedef struct CamDemarcateParam{
	unsigned short cam2stop;
	unsigned short camheight;
	//unsigned short lannum;    //������
	//unsigned short number;    //ͨ�����
	unsigned short baselinelen;
	unsigned short farth2stop;
	unsigned short recent2stop;
}mCamDemarcateParam; //����궨����

typedef struct ChannelVirtualcoil{
	unsigned short number;      //ͨ�����
	unsigned short farthCoillen;
	unsigned short recentCoillen;
}mChannelVirtualcoil; //ͨ��������Ȧ�Ĳ���

typedef struct CamParam{
	unsigned char coilnum;     //ͨ����
	mCamAttributes camattr;
	mCamDemarcateParam camdem;
	mChannelVirtualcoil channelcoil[DETECTLANENUMMAX];
}mCamParam;

//---------------��Ȧ����------�����������������---------
typedef struct Point{
    unsigned short x;
    unsigned short y;
}mPoint;//������

typedef struct Line{
    unsigned short startx;
    unsigned short starty;
    unsigned short endx;
    unsigned short endy;
}mLine; //������

typedef struct RearCoil{
    int landID;//����ID
    unsigned char Landtype; //车道类型�?为竖向（垂直），1为横向（水平�?
	mPoint RearCoil[COILPOINTMAX];  //ռλ�����Ȧ
	mPoint MiddleCoil[COILPOINTMAX];
	mPoint FrontCoil[COILPOINTMAX]; //ǰ����Ȧ
}mChannelCoil;  //��ͨ��������Ȧ��λ��

typedef struct CamDetectLane{
	unsigned char lanenum;                 //������
	mChannelCoil virtuallane[DETECTLANENUMMAX];
}mCamDetectLane;   //���������صĳ�������, ��������ͨ��������Ȧ //ÿһ����������������Ȧ

typedef struct VirtualLaneLine{
	unsigned char lanelinenum;         //
	mLine         laneline[LANELINEMAX];
}mVirtualLaneLine;    //���⳵���� ���֧��4������5����

typedef struct StandardPoint{
	mPoint  coordinate;
	unsigned short value;
}mStandardPoint; //�궨��������ֵ

typedef struct DemDetectArea{
	mPoint  vircoordinate[DETECT_AREA_MAX];
	mPoint  realcoordinate[DETECT_AREA_MAX];
}mDemDetectArea;  // �궨����ϵ8����,������ʵ�������

typedef struct DetectParam{
	unsigned short uTransFactor;
	unsigned int   uGraySubThreshold;
	unsigned int   uSpeedCounterChangedThreshold;
	unsigned int   uSpeedCounterChangedThreshold1;
	unsigned int   uSpeedCounterChangedThreshold2;
	unsigned short  uDayNightJudgeMinContiuFrame;//�л�ʱ��ֵ����ֵ
	unsigned short  uComprehensiveSens;//ȡ����������֡��
	unsigned short  uDetectSens1;//�ж��ǳ�ͷ����С����
	unsigned short  uDetectSens2;
	unsigned short  uStatisticsSens1;
	unsigned short  uStatisticsSens2;	//by david 20130910 from tagCfgs
	unsigned short  uSobelThreshold;//sobel��ֵ
	unsigned short  shutterMax;        // 1 2 3 4 5 6 7 8
	unsigned short  shutterMin;        // 1 2 3 4 5 6 7 8
}mDetectParam;


//ϵͳ��Ϣ
typedef struct{
	char devicetype[32];
	char firmwareV[32];
	char webV[32];
	char libV[32];
	char hardwareV[32];
	//char serial[32];
}IVDDevInfo;

//������Ϣ����
typedef struct{
    char checkaddr[50];
    char devUserNo[8];
    char undefine2;
    uint8 autoreset;
    uint8 overWrite;
    uint8 loglevel;
    uint32 timeset;
    uint8  pro_type;
}IVDDevSets;

//ʱ���������
typedef struct{
	unsigned short year;
	unsigned char month;
	unsigned char date;
	unsigned char hour;
	unsigned char minute;
	unsigned char second;
}TimeSetInterface;
//ntp
typedef struct{
	char   ipaddr[16];
	uint32 port;
	uint32 cycle;
}IVDNTP;

typedef struct {
unsigned char cam_num;
char exist[CAM_MAX];  //0--������    1---����
}IVDCAMERANUM;

//2.5.�����������
typedef struct{
    char strIpaddr[16];
    char strIpaddr1[16];
    char strIpaddr2[16];
    char strIpaddrIO[16];
    uint32 strPort;
	uint32 strPortIO;
    char   UpServer;
    char   strNetmask[16];
    char   strGateway[16];
    char   strMac[20];
    uint32 tcpPort;
    uint32 udpPort;
    uint16 maxConn;
    char strDNS1[16];
    char strDNS2[16];
}IVDNetInfo;

//2.6.�л�ʱ������ //0 1�����賿 ��1����2   2 3���� �ƻ� ��1����2
typedef struct{
	unsigned int timep1;
	unsigned int timep2;
	unsigned int timep3;
	unsigned int timep4;
}IVDTimeStatu;

//���ڲ�������
typedef struct{
    unsigned short  uartNo;       //uart number  // 0: com0 1:com1����
    unsigned short  protocol;     //protocol type  // 1- Ӧ�� 2-����
    unsigned short  buadrate;     //rate of buad //0
    char   databit;      // data bit 8λ
    char   stopbit;      // 1
    char   checkbit;     // 0 �� 1-�� 2-ż
}RS485CONFIG;

//ͳ�Ʋ�������
typedef struct{
    uint16 period;
    uint8  type;
    uint8  tiny;
    uint8  small;
    uint8  mediu;
    uint8  large;
    uint8  huge;
}IVDStatisSets;

//Э���������
typedef struct{
    uint8  type;
}mThirProtocol;

typedef struct PersonArea{
unsigned char id;//����id
mPoint  realcoordinate[4];
mPoint detectline[2];//���˼������,0��������1�յ�����
}mPersonArea;

typedef struct PersonDetectArea{
unsigned char num;//��������
mPersonArea area[AREAMAX];
//mPoint detectline[2];//���˼������,0��������1�յ�����

}mPersonDetectArea; //���˼������

typedef struct Otherparam{
unsigned char  directio;//��ⷽ��
unsigned char detecttype;//�������
unsigned char videotype;//��ƵԴ
char rtsppath[FILEPATHMAX]; //rtsp��ַ
unsigned char  detectaccuracy;//��⾫��
unsigned short personlimit;//��������ֵ
unsigned char  pensondetecttype;//���˼������
char  camIp[IPADDRMAX];//���ip
unsigned int   camPort;//���ip
char  username[USERNAMEMAX];
unsigned char  passwd[USERNAMEMAX];
unsigned int camerId;
unsigned char camdirection;//相机方向，方向定义见上宏定义
char filePath[FILEPATHMAX];
}mOtherparam ;//���͵���������

typedef struct CamDetectParam{
	mCamDetectLane detectlane;        //��⳵������
	mVirtualLaneLine  laneline;      //������
	mStandardPoint standpoint[STANDPOINT];       //�궨�������
	mDemDetectArea area;              //�궨������
	mDetectParam detectparam[ALGMAX];   // ��������0  ����Ĳ���,  1�������ϲ���
    mPersonDetectArea personarea;        //���˼������
    mOtherparam other;  //���Ͳ�������������
    mCamDemarcateParam camdem; //����궨����
}mCamDetectParam;  //�㷨���м���йز���

typedef struct{
unsigned char id;//����id   
unsigned int time;//���˵ȴ�ʱ��
}mRealPersonTestInfo;


typedef struct
{
unsigned int    lagerVehnum;//������
unsigned int    smallVehnum;//С������
unsigned int    Vehnum;//������
unsigned int    speed;//ʵʱ�ٶ�
unsigned int    aveSpeed;//ƽ���ٶ�
unsigned short  queueLength;//�Ŷӳ���
unsigned int    timedist;//ƽ����ͷʱ��
unsigned int    share;//ƽ��ʱ��ռ����
}mRealLaneTestInfo;

typedef struct{
  unsigned char   laneNum;       //ʵ�ʳ�������
  mRealLaneTestInfo lane[DETECTLANENUMMAX]; //��������DETECTLANENUMMAX=8
  mRealPersonTestInfo person[TEST_PERSON_AREA_SIZE];
}mRealTestInfo;

typedef struct{
    mRealStaticInfo static_info[CAM_MAX];
    unsigned int pre_car_num[CAM_MAX][DETECTLANENUMMAX][2];
    unsigned int car_num[CAM_MAX][DETECTLANENUMMAX][2];
    unsigned char reset_flag[CAM_MAX];
    mRealTestInfo real_test_info[CAM_MAX];
    mRealTestInfo real_test_one[CAM_MAX];
    mRealTestInfo real_test_five[CAM_MAX];
    unsigned char real_test_updated[CAM_MAX];
    pthread_mutex_t lock[CAM_MAX];
    pthread_mutex_t real_test_lock[CAM_MAX];
}EX_mRealStaticInfo;

typedef struct {
    mCommand        pack_head;
    mRealStaticInfo static_info;
} real_data_pack_t;

typedef struct {
    mCommand        pack_head;
    mRealTestInfo   real_test_info;
} real_test_data_pack_t;

typedef struct
{
	unsigned char   plannum;//ʱ��α��
	unsigned char   start_hour;//��ʼʱ��-Сʱ
	unsigned char   start_minute;//��ʼʱ��-����
	unsigned char   personlimit;//�ȴ�����������ֵ
	unsigned char   maxWaitTime;//�������ʱ��
	unsigned short  noPersonTime;//�������ʱ��
	unsigned short  overTime;//��ʱ
}mAreaPlanInfo;

typedef struct{
  unsigned char areaNum; //���˼��������
  unsigned char planTotal; //ʱ�����
  mAreaPlanInfo plan[48]; //���ʱ�����48��
}mPersonPlanInfo;

typedef struct {
    unsigned char perso_num[PERSON_AREAS_MAX]; //ʵʱ����
    unsigned char up_perso_num[PERSON_AREAS_MAX]; //��������
    unsigned char down_perso_num[PERSON_AREAS_MAX]; //��������
    long long   no_person_time[PERSON_AREAS_MAX]; //û�����˵�ʱ��
    long long   gj_no_person_time[PERSON_AREAS_MAX];
    long long   gj_have_person_time[PERSON_AREAS_MAX]; //��������ʱ��
    long long   line_no_person_time[PERSON_AREAS_MAX]; //û�����˵�ʱ��
    unsigned int  prev_status[PERSON_AREAS_MAX]; //��һ��״̬���Ƿ�����
    unsigned char work_staus; //��Ƶ�쳣���ܼ��ȵ�
    long long     start_over_person_limit_ms[PERSON_AREAS_MAX];
	long long     start_unover_person_limit_ms[PERSON_AREAS_MAX];
	long long     start_wait_ms[PERSON_AREAS_MAX];//�ȴ�ʱ��second
	//unsigned int  prev_status_start_time[PERSON_AREAS_MAX];//��һ����Ҫ�仯��״̬
	unsigned char prev_send_area_status[PERSON_AREAS_MAX];
	unsigned char prev_type[PERSON_AREAS_MAX];
	pthread_mutex_t lock;
}mRealTimePerson;

typedef struct {
    unsigned char area_no;
    unsigned char area_status;
    unsigned char occupt;
    unsigned char person_status;
    unsigned char work_status;
    unsigned char area_person_num;
    unsigned int  wait_sec;
}mPersonCheckData;

/*
typedef struct Point{
    unsigned short x;
    unsigned short y;
}mPoint;
*/
typedef union
{
unsigned int type;
struct
{
unsigned char bit0:1;
unsigned char bit1:1;
unsigned char bit2:1;
unsigned char bit3:1;
unsigned char bit4:1;
unsigned char bit5:1;
unsigned char bit6:1;
unsigned char bit7:1;
unsigned char bit8:1;
unsigned char bit9:1;
unsigned char bit10:1;
unsigned char bit11:1;
unsigned char bit12:1;
unsigned char bit13:1;
unsigned char bit14:1;
unsigned char bit15:1;
unsigned char bit16:1;
unsigned char bit17:1;
unsigned char bit18:1;
unsigned char bit19:1;
unsigned char bit20:1;
unsigned char bit21:1;
unsigned char bit22:1;
unsigned char bit23:1;
unsigned char bit24:1;
unsigned char bit25:1;
unsigned char bit26:1;
unsigned char bit27:1;
unsigned char bit28:1;
unsigned char bit29:1;
unsigned char bit30:1;
unsigned char bit31:1;
}bits;
}mSelectType;

typedef struct
{
unsigned char      areaNum;//������
mPoint  realcoordinate[4];//��������
mSelectType  eventType;//�¼�����
unsigned int  reserve;//Ԥ��
unsigned char   direction;//��������
unsigned char report[32] ;//�¼����ʱ����
unsigned char reserve1[32];//Ԥ��
}mEventArea;

typedef struct{
  unsigned char   eventAreaNum; //�¼���������
  mEventArea  eventArea[8]; //���8�¼���������
}mEventInfo;

typedef struct{
unsigned char eventType;//�¼�����
mPoint eventRect[4];//�¼�Ŀ�������
unsigned char picPath[100];//ͼƬ·��
unsigned char videoPath[100];//��Ƶ·��
}mEventPara;

typedef struct{
unsigned char   flag;          //�¼����ݱ�־0xFE
unsigned int  deviceId;//�豸ID
unsigned int  camId;//���ID
unsigned char ereaId;//����ID
unsigned int eventId;//�¼�id
unsigned char newEventFlag;//���¼���־
unsigned char eventNum;//�¼�����
mEventPara eventData[32];
}mRealEventInfo;


#pragma pack(pop)

enum{
	CLASS_NULL,
	CLASS_char,
	CLASS_short,
	CLASS_int,
	CLASS_mCommand,
	CLASS_mBaseInfo,
	CLASS_mAlgInfo,
	CLASS_mDate,
	CLASS_mNTP,
	CLASS_mSysInfo,
	CLASS_mNetworkInfo,
	CLASS_mChangeTIME,
	CLASS_mSerialInfo,
	CLASS_mStatiscInfo,
	CLASS_mProtocolInfo,
	CLASS_mCameraStatus,
	CLASS_mCameraDelete,
    CLASS_mPersonAreaTimes,
    CLASS_mEventInfo
};
void net_decode_obj(unsigned char *bf,int type,int encode);
void net_decode_obj_n(unsigned char *addr,int type,int encode,int num,int size);
int get_obj_len(int class_type);
int prepare_pkt(unsigned char *p_start, int head_length,int reply_type, int class_type, int class_length,unsigned char *p_obj);
int handle_pkt(unsigned char *p_start, int head_length, int class_type, int class_length);
int get_pkt(unsigned char *p_start, int head_length,int reply_type, int class_type, int class_length,unsigned char *p_obj);

#endif /* PROTOCOL_H_ */
