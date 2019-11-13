/*******************************************************************************
Copyright (c) HKVision Tech. Co., Ltd. All rights reserved.
--------------------------------------------------------------------------------

Date Created: 2011-10-25
Author: wuxiaofan
Description: ����ͨѶ������ݻ�ȡSDKͷ�ļ�������PTZ�ͼ��������ķ�װ

--------------------------------------------------------------------------------
Modification History
DATE          AUTHOR          DESCRIPTION
--------------------------------------------------------------------------------
YYYY-MM-DD

*******************************************************************************/

#ifndef _INF_SDK_NET_H_
#define _INF_SDK_NET_H_
 
#define DO_NOTHING

#if defined(WIN32)
	#ifndef INLINE
		#define INLINE __inline
	#endif
#else
#define INLINE inline
#endif


/*use for parameter INPUT, *DO NOT Modify the value* */
#define IN
/* use for parameter OUTPUT, the value maybe change when return from the function 
 * the init value is ingore in the function.*/
#define OUT
/*use for parameter INPUT and OUTPUT*/
#define IO

/* --------------------------------  */
#define EXTERN extern
#define STATIC static

#define LOCALVAR static
#define GLOBALVAR extern


/*����ȫ�ֱ���ʱ*/
#define DECLARE_GLOBALVAR 

/*ʹ��ȫ�ֱ���ʱ, �ô�����*/
#define USE_GLOBALVAR extern
#define LOCALFUNC    static
#define EXTERNFUNC   extern

/*�Ͳ�ε�LOW API*/
#define LAPI  
/*�߲��API*/
#define HAPI  
/*Multimedia Frame API*/
#define MMFAPI

/* -------- Standard input/output/err *****/
#define STDIN  stdin
#define STDOUT stdout
#define STDERR stderr
#if 0
//MARCO define
#define SAFE_DELETE(p)  { if(p != NULL) { delete (p);     (p) = NULL; } }   //Delete object by New create 
#define SAFE_DELETEA(p) { if(p != NULL) { delete[] (p);   (p) = NULL; } }   //Delete Arrary
#define SAFE_RELEASE(p) { if(p != NULL) { (p)->Release(); (p) = NULL; } }
#define SAFE_FREE(p)	{ if(p != NULL) { free(p); (p) = NULL; } }
#endif

#define SOCKET_ERROR            (-1)
#define INVALID_SOCKET -1
#define WEAK __attribute__((weak))
#define Packed  __attribute__((aligned(1),packed))

#define MAX_FILENAME_LEN 128
#define AUDIO_SAMPLE_RATE 8000
#define AUDIO_QUANTSIZE 16
#define AUDIO_CHANNELS 1


#ifndef    NULL
#define	   NULL 0
#endif
#define TRUE 1
#define FALSE 0

typedef unsigned long       DWORD;
typedef unsigned short      WORD;
typedef void *LPVOID;
//typedef int BOOL;
#ifndef BOOL
#define BOOL int
#endif
typedef unsigned char BYTE;

typedef unsigned int        UINT32, *PUINT32;

typedef unsigned int UINT;

typedef void VOID;

/******************************************************************************
SDKNET�����붨�壬��ӦNET_IPC_GetLastError�ӿڵķ���ֵ
*******************************************************************************/
//ͨ�ô���
#define ERR_SUCCEED         0           /**< ִ�гɹ� */
#define ERR_FAIL            -1          /**< ִ��ʧ�� */
#define ERR_INVALIDPARAM    0x80000001  /**< ��������Ƿ� */
#define ERR_NOMEMORY        0x80000002  /**< �ڴ����ʧ�� */
#define ERR_SYSFAIL         0x80000003  /**< ϵͳͨ�ô��� */
#define ERR_USERNAME        0x80000004  /**< �û������� */
#define ERR_PASSWORD        0x80000005  /**< ������� */
#define ERR_NOINIT          0x80000006  /**< û�г�ʼ�� */
#define ERR_INVALIDCHANNEL  0x80000007  /**< ͨ���Ŵ��� */
//�������
#define ERR_OPENSOCKET      0x80000008  /**< ����SOCKET���� */
#define ERR_SEND            0x80000009  /**< ���豸������������ʧ�� */
#define ERR_RECV            0x80000010  /**< ���豸������������ʧ�� */
#define ERR_CONNNECT        0x80000011  /**< �����豸ʧ�ܣ��豸�����ߡ��豸æ����
										*��ԭ����������ӳ�ʱ�� */


//�汾
#define IP_VERSION4					4			//IPV4
#define IP_VERSION6					6			//IPV6

//����Ƶ����
#define DATA_VIDEO					1			//��Ƶ��
#define DATA_AUDIO					2			//��Ƶ��
#define DATA_AV						3			//������

//��Ƶ��������
#define VIDEO_MPEG4_MAJOR			1			//MPEG4������
#define VIDEO_MPEG4_MINOR			2			//MPEG4������
#define VIDEO_MJPEG					3			//MJPEG����
#define VIDEO_H264_MAJOR			4			//H264������
#define VIDEO_H264_MINOR			5			//H264������

//��Ƶ��������

#define	AUDIO_G711_A				0x02		//G711_A
#define	AUDIO_G711_U				0x01		//G711_U
#define	AUDIO_ADPCM_A				0x03		//ADPCM_A
#define	AUDIO_G726					0x04		//G726
#define	AUDIO_G711_A_HI				0x05		//HI H264
#define	AUDIO_G711_U_HI				0x06		//HI H264
#define	AUDIO_G726_HI				0x07		//HI H264

//֡����
#define FRAME_VOL					0xD0		//VOL
#define FRAME_IVOP					0xD1		//I֡
#define FRAME_PVOP					0xD2		//P֡
#define FRAME_AUDIO					0xD3		//��Ƶ֡

//��¼���������
#define RE_SUCCESS					0			//�ɹ�
#define RE_USERNAME_ERROR			1			//�û�������
#define RE_PASSWORD_ERROR			2			//�������


//��Ƶ���������
#define RE_AV_SUCCESS               0           //����
#define RE_AV_FULL_ERROR            1           //��������
#define RE_AV_LOST_ERROR            2           //��Ƶ��ʧ
#define RE_AV_TYPE_ERROR            3           //������Ƶ����
#define RE_AV_NONSUPPORT_ERROR      4           //��֧�� 
#define RE_AV_NO_PRIVILEGE          5           //û��Ȩ��

#define DEVICE_TYPE_ASIC	0x00
#define DEVICE_TYPE_DSP		0x01
#define DEVICE_TYPE_H264	0x02
#define DEVICE_TYPE_DM355	0x03
#define DEVICE_TYPE_POWERPC	0x04
#define DEVICE_TYPE_HI3510	0x05
#define DEVICE_TYPE_DM365   0x06
#define DEVICE_TYPE_HI3512  0x07
#define DEVICE_TYPE_MG3500  0x08
#define DEVICE_TYPE_DM368	0x09
#define DEVICE_TYPE_3061	0x0A


#define TEXT_LENGTH					32

//��Ƶ��������
#define PLAY_AUDIO_SAMPLE_POOR		8000
#define PLAY_AUDIO_SAMPLE_LOW		11025
#define PLAY_AUDIO_SAMPLE_NORMAL	22050
#define PLAY_AUDIO_SAMPLE_HIGH		44100

//ת������ͻ�������
#define	CLIENT_DECODER				0x01		//DECODER
#define	CLIENT_LMC					0x02		//LMC
#define	CLIENT_SMT					0x03		//SMT
#define	CLIENT_NVR					0x04		//NVR
#define CLIENT_WEB					0x05		//Web 

/******************************************************************************
SDKNET���ݽṹ����
*******************************************************************************/
/**
* @struct tagPlayParam
* @brief �������Ͳ���
* @attention
*/
typedef enum tagEncodeType
{
	ENCODE_MPEG4 = 1,   /**< MPEG4���� */
	ENCODE_H264,        /**< H264���� */
	ENCODE_H264_Hi3510, /**< H264 3510���� */
	ENCODE_MJPEG,       /**< MJPEG���룬������ */
}E_ENCODE_TYPE;

/**
* @struct tagRealDataInfo
* @brief ʵʱ����������
* @attention
*/
typedef struct tagRealDataInfo
{
	unsigned long lChannel;    /**< ͨ���ţ���0��ʼ */
	unsigned long lStreamMode; /**< �������ͣ�0-��������1-������ */
	unsigned int eEncodeType; /**< �������� */
}S_REALDATA_INFO;

typedef struct{
	int iChannel;		/**<ͨ���Ŵ�0��ʼ*/
	int iAVType;		/**<����Ƶ����:1~��Ƶ��2~��Ƶ��3~������*/
	int iEncodeType;	/**<��������:3~Mjpeg 4~������5~������*/
}S_REALPLAY_INFO;

/**
* @enum tagRealDataType
* @brief �ص�ʵʱ������������
* @attention ��
*/
typedef enum tagRealDataType
{
	REALDATA_HEAD,   /**< ʵʱ����ͷ���� */
	REALDATA_VIDEO,  /**< ʵʱ��Ƶ�����ݣ�����������������Ƶ�ֿ�����Ƶ�����ݣ� */
	REALDATA_AUDIO,  /**< ʵʱ��Ƶ������ */
}E_REALDATA_TYPE;

/****************************************************************
** ���ݽṹ��: SnapShotParamInfo
** ��������:  ץ��ͼƬ������Ϣ,���ڻ�ȡ������
** �� ��:      
** �� ��:      2012-2-9
****************************************************************/
typedef struct tagSnapShotParamInfo
{
	unsigned char	cCmdType;				 //1:MJPEG���գ�2:������������
	unsigned char	cPhotoNum;               //ץ������ 1-4
	unsigned char	usOutTouchNum[2];        //usOutTouchNum[0]:�ⴥ����ʱ1   0ms-200ms  usOutTouchNum[1]:�ⴥ����ʱ2   0ms-200ms
	unsigned char	usPhotoSpaceNum;         //���ռ��      20-100   ��*10��ms   ��ֵ��Ҫ��10
	unsigned char	cGraspPhotoType;         //1:���  2:���� 3:����
	unsigned char	cTouchType;				 //1:��ƽ     2:�ź���
	unsigned char   cSignalType;             //1:������   2:�½���
	unsigned char   cPhotoLampType[4];       //cPhotoLampType[0]:��һ��  1:��  2:��  ������Դ�����  
	unsigned char   cLampType;               //1:�����   2:�����  
	unsigned char   cShutterModel;           //1:�Զ� 2:�ֶ���Ĭ���Զ�
	unsigned char   cStreamExpTime;          //��Ƶ�ع�ֵ 0-240 Ĭ��240 ����1
	unsigned char   cAGC;                   //�Զ����� 0-180 Ĭ��100 ����1
	unsigned char   cEmpNum;                 //ʹ����ʱ  0-200   ��*10��ms
	unsigned char   cSnapExpTime;       // ץ��˲���ع�ֵ 0-240 Ĭ��0 ����1
	unsigned char   cFill[2];              //����ֽڣ��չ�4�ֽ�
	unsigned short  sDistence[4];            //���� ����
	unsigned short  sTriggerDelay;      // �������ʱ 0-60000 Ĭ��38500 ����600 
	unsigned short  sTriggerWidth;      //������ 1-4000 Ĭ��1000 ����40
	unsigned char   cReserve[16];              //�����ֽڣ���չ��
}SnapShotParamInfo, *pSnapShotParamInfo;

typedef struct tagSpeedInfo
{
	BYTE ID;
	BYTE CLane;
	BYTE Fill[2];//����ֽ�
	unsigned short usSpeed[4];
}Packed SpeedInfo;

#define PHOTO_NAME_LEN  64  //ͼƬ���Ƴ���

typedef struct tagSnapShotPhotoInfo
{
	SpeedInfo     tSpeedInfo;       //������Ϣ
	unsigned char cPhotoName[PHOTO_NAME_LEN]; // ͼƬ���ƣ�PHOTO_NAME_LEN����Ϊ4�ı���
	unsigned int  uPhotoDataLen;    //ͼƬ��С���ֽ���
}SnapShotPhotoInfo;

typedef struct tagPhotoData
{
	BYTE              cSaveFlag;      //0:������ 1:���棬Ĭ��ֵ
	BYTE			  cFill[3];		 // ���
	char *			  cPhotoPath; //ͼƬ����·�� 
	SnapShotPhotoInfo tPhotoInfo; //ͼƬ��Ϣ
	BYTE*			  pPhotoData;              //ͼƬ����ָ��
}PhotoData; 

/**
* @struct tagTalkParam
* @brief �����Խ��Ĳ���
* @attention
*/
typedef struct tagTalkParam
{
	unsigned int nAudioEncode;    /**< Ԥ������Ƶ�������� */
	unsigned int nSamplesPerSec;  /**< ����Ƶ�ʣ�ȡֵΪ��8000��11025��22050��44100 */
	unsigned int nBitsPerSample;  /**< Ԥ��������λ�����磺8��16 */
}S_TALK_PARAM;

/**
* @struct tagAlarmerInfo
* @brief ����Դ�豸��Ϣ
* @attention ��
*/
typedef struct tagAlarmerInfo
{   
	char sDeviceIP[128];      /**< ����Դ�豸��IP��ַ */
	unsigned short wLinkPort; /**< ����Դ�豸��ͨѶ�˿� */
}S_ALARMER_INFO;

/**
* @enum tagRealDataType
* @brief ��������
* @attention ��
*/
typedef enum tagAlarmType
{
	ALARM_UNKNOWN = 0,/**< δ֪���ͱ��� */
	ALARM_INPUT,      /**< �̵������뱨�� */
	ALARM_MOTION,     /**< �ƶ���ⱨ�� */
	ALARM_SHELTER,    /**< ��Ƶ�ڵ����� */
	ALARM_VIDEOLOST,  /**< ��Ƶ��ʧ���� */
	ALARM_DEVICEERR,  /**< Ԥ�����豸�쳣���� */
}E_ALARM_TYPE;
/**
* @struct tagAlarmerDeviceInfo
* @brief ������Ϣ
* @attention ��
*/
typedef struct tagAlarmInfo
{
	E_ALARM_TYPE eAlarmType;    /**< �������� */
	unsigned int nAlarmID;      /**< ������ʶ�ţ��豸�쳣ʱ��ʾ�쳣���ͣ���
								*1��ʼ���̵������뱨����1��ʾ�̵��������1��
								*�ƶ���ⱨ����1��ʾ�ƶ����1�ȡ��豸�쳣��
								*�ͣ�1-Ӳ������2-Ӳ�̳���3-����Ͽ���4-��
								*�����ʣ�5-�����ͻ */ 
	unsigned char cAlarmStatus; /**< ����״̬��0-�ޱ�����1-�б������̵���
								*���뱨��ʱ�� 0-����ȡ����1-����������2-��
								*������ */
	unsigned char cAlarmArea;   /**< ��������ţ��ƶ�������Ƶ�ڵ���Ч��
								*����Ŵ����ĸ����������� */
}S_ALARM_INFO;

/*justin add 2014.1.14*/
typedef enum tagSerialReqType
{
	SERIAL_SWITCH_TYPE,
	SERIAL_CONNECT_TYPE,
	SERIAL_SET_TYPE,
	SERIAL_SEND_TYPE,
	SERIAL_MAX_TYPE,
}E_REQ_SERIAL_TYPE;

/**
*@enum tagPtzType
*@brief ��̨��������:  �ƶ�+  3D��λ
*/
typedef enum tagPtzType
{
	PTZ_MOVE_TYPE,
	PTZ_3D_TYPE,
	END_TYPE,
}E_PTZ_TYPE;

/**
*@enum tagActType
*@brief 3D��λʱ��������
*@�������¿�ѡΪ�Ŵ󣬴������Ͽ�ѡ����С
*/
typedef enum tagActType
{
	Click = 1,		/**< ���Ŵ�*/
	DblClick,			/**< �Ŵ�4��*/
	ZoomIn,			/**< �Ŵ�*/
	ZoomOut		/**< ��С*/
} E_ActionType;

/**
* @enum tagPtzCommand
* @brief ��̨��������
* @attention ͬʱ������NET_IPC_PTZControl�ӿ���2��������Ӧ�ĺ�������ã�p1��ʾ
*����iParam1��p2��ʾ����iParam2
*/
typedef enum tagPtzCommand
{    
	//��������
	ZOOM_TELE,      /**< ������(���ʱ��,��Ұ��С,Ŀ��Ŵ�),p1�ٶ� */
	ZOOM_WIDE,      /**< �����С(���ʱ�С,��Ұ�Ŵ�,Ŀ����С),p1�ٶ� */
	FOCUS_NEAR,     /**< ����ǰ��(Ŀ�꿿��),p1�ٶ� */
	FOCUS_FAR,      /**< ������(Ŀ��Զ��),p1�ٶ� */
	IRIS_OPEN,      /**< ��Ȧ����,p1�ٶ� */
	IRIS_CLOSE,     /**< ��Ȧ��С,p1�ٶ� */
	UP,             /**< ��ת,p1�ٶ� */
	DOWN,           /**< ��ת,p1�ٶ� */
	LEFT,		    /**< ��ת,p1�ٶ� */
	RIGHT,		    /**< ��ת,p1�ٶ� */
	UP_LEFT,		/**< ����,p1�ٶ� */
	UP_RIGHT,		/**< ����,p1�ٶ� */
	DOWN_LEFT,		/**< ����,p1�ٶ� */
	DOWN_RIGHT,		/**< ����,p1�ٶ� */

	//Ԥ��λ����
	SET_PRESET,     		/**< ����Ԥ�õ�,p1Ԥ�õ�����(1-255) */
	CALL_PRESET,    		/**< ת��Ԥ�õ�,p1Ԥ�õ�����  */

	//����ɨ��
	START_PATTERN,   	/**< ��ʼ����ɨ��,p1����ɨ������(1-4) */
	STOP_PATTERN,    	/**< ֹͣ����ɨ��,p1����ɨ������ */
	RUN_PATTERN,     	/**< ���л���ɨ��,p1����ɨ������ */

	//�Զ�ˮƽ����
	START_AUTO_PAN, /**< ��ʼ�Զ�ˮƽ����,p1�Զ�ˮƽ���е����(1-4) */
	STOP_AUTO_PAN,  /**< ֹͣ�Զ�ˮƽ����,p1�Զ�ˮƽ���е���� */
	RUN_AUTO_PAN,   /**< �����Զ�ˮƽ����,p1�Զ�ˮƽ���е���� */

	AUTO_SCAN,      /**< �Զ�ɨ�� */
	FLIP,           /**< ��ת */
	STOP,           /**< ֹͣ */
	VECTOR,		/**<ʸ������*/
	PTZ_3D,	/**<3D ��λ*/
	CMD_END,
}E_PTZ_COMMAND;

/*justin add 2014.1.14*/
/*justin add 2014.1.14*/
/**
*@ brief - ͸��ͨ������󷵻ؽ������ֵ
*/
typedef enum Serialresponsecode_Eum
{
	SerialStatusOK=0,
	SerialReceiveData=1,
	SerialUnsupport,
	SerialDeviceBusy,
	SerialChanelError,
	SerialSendingError,
	SerialBitsError,
	SerialParityError,
	SerialStopBitError,
	SerialBaudrateError,
	SerialFail
}Serialresponsecode_Eum;


#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
SDKNET��ʼ���ӿ�
*******************************************************************************/
/**
* ��ʼ��SDK����������SDK������ǰ��
* @return �������½����
* - �ɹ���true
* - ʧ�ܣ�false
* - ��ȡ���������NET_IPC_GetLastError
* @note ��
*/
BOOL NET_IPC_Init();
/**
* �ͷ�SDK��Դ���ڽ���֮ǰ������
* @return �������½����
* - �ɹ���true
* - ʧ�ܣ�false
* - ��ȡ���������NET_IPC_GetLastError
* @note ��
*/
BOOL NET_IPC_Cleanup();
/******************************************************************************
SDKNET��ȡ������ӿ�
*******************************************************************************/
/**
* ��ȡ������
* @return ����ֵΪ������
* @note ��
*/
long NET_IPC_GetLastError();

/******************************************************************************
SDKNET�û�ע��ӿ�
*******************************************************************************/
/**
* �û�ע��
* @param [IN]   sDevIP    �豸IP��ַ
* @param [IN]   nDevPort  �豸�˿ں�
* @param [IN]   sUserName ��¼���û���
* @param [IN]   sPassword �û�����
* @return �������½����
* - ʧ�ܣ�-1
* - ����ֵ����ʾ���ص��û�IDֵ�����û�ID����Ψһ�ԣ��������豸�Ĳ�������Ҫͨ����IDʵ��
* - ��ȡ���������NET_IPC_GetLastError
* @note ��
*/
long NET_IPC_Login(const char         *sDevIP,
				  const unsigned int nDevPort,
				  const char         *sUserName,
				  const char         *sPassword);
/**
* �û�ע��
* @param [IN]   lLoginID �û�ID�ţ�NET_IPC_Login�ķ���ֵ
* @return �������½����
* - �ɹ���true
* - ʧ�ܣ�false
* - ��ȡ���������NET_IPC_GetLastError
* @note ��
*/
BOOL NET_IPC_Logout(long lLoginID);

/******************************************************************************
SDKNETʵʱ����ȡ�ӿ�
*******************************************************************************/

/**
* ��ʼʵʱ���ݻ�ȡ
* @param [IN]   lLoginID      ��½��ID��NET_IPC_Login�ķ���ֵ
* @param [IN]   sRealDataInfo ʵʱ�������Ĳ����ṹ��
* @param [IN]   fRealData     �������ݻص�����
* @param [IN]   pUserData     �û��Զ�������ݣ��ص�����ԭֵ����
* @return �������½����
* - ʧ�ܣ�-1
* - ����ֵ����ΪNET_IPC_StopRealData�Ⱥ����ľ������
* - ��ȡ���������NET_IPC_GetLastError
* @note ��
*/
typedef void(*CBRealData)(int iStreamID,unsigned char  *pFrameData,int iFrameSize,void *pUserData);


/**
* ֹͣʵʱ���ݻ�ȡ
* @param [IN]   lRealHandle ��½��ID��NET_IPC_Login�ķ���ֵ
* @return �������½����
* - �ɹ���true
* - ʧ�ܣ�false
* - ��ȡ���������NET_IPC_GetLastError
* @note ��
*/
BOOL NET_IPC_StopRealData(long lRealHandle,int iStreamID);

/**
* ֹͣʵʱ���ݻ�ȡ
* @param [IN]   lRealHandle ��½��ID��NET_IPC_Login�ķ���ֵ
* @return �������½����
* - �ɹ���true
* - ʧ�ܣ�false
* - ��ȡ���������NET_IPC_GetLastError
* @note ��
*/
BOOL NET_IPC_CloseRealDataConnect(long lRealHandle,int iStreamID,int iSockfd);


/******************************************************************************
SDKNET��̨���ƽӿ�
*******************************************************************************/
/**
* ��̨���ƽӿڣ���������Ԥ��ʱҲ����ʹ��
* @param [IN]   lLoginID    ��½��ID��NET_IPC_Login�ķ���ֵ
* @param [IN]   nChannel    �豸ͨ���ţ� ��0��ʼ
* @param [IN]   ePTZCommand ��̨��������
* @param [IN]   iParam1     ����1���������ݸ����������йأ����E_PTZ_COMMAND
* @param [IN]   iParam2     ����2��ͬ��
* @param [IN]   iParam3     ����3��ͬ��
* @param [IN]   iParam4     ����4��ͬ��
* @param [IN]   iParam5     ����5��ͬ��
* @param [IN]   iParam6     ����6��ͬ��
* @param [IN]   iParam7     ����7��ͬ��
* @param [IN]   cRes          ����
* @return �������½����
* - �ɹ���true
* - ʧ�ܣ�false
* - ��ȡ���������NET_IPC_GetLastError
* @note ��iParam1��ʾ�ٶ�ʱ����Χ��1~8
*/
BOOL  NET_IPC_PTZControl(long          lLoginID,
						unsigned int  nChannel,
						E_PTZ_COMMAND ePTZCommand,
						int           iParam1 /*= 0*/,
						int           iParam2/* = 0*/,
						int           iParam3/* = 0*/,
						int           iParam4/* = 0*/,
						int           iParam5/* = 0*/,
						int           iParam6/* = 0*/,
						int           iParam7/* = 0*/,
						char cRes /*= 0*/);


typedef BOOL(*CBTransData)(int result, BYTE *data, int datalen);

/**
* @brief - ͸��ͨ�����ƽӿ�
* @param[in] lLoginID  		 ��½��ID��NET_IPC_Login�ķ���ֵ
* @param[in] nChannel		 ͨ���ţ�Ĭ��Ϊ1
* @param[in] eSerialReqType͸��ͨ����������
* @param[in] iParam1 		 �������ݸ����������йأ�����ĵ�˵��
* @param[in] iParam2 		 ͬ��
* @param[in] iParam3 		 ͬ��
* @param[in] iParam4 		 ͬ��
* @param[in] data		 ͨ�����ڷ��͵�����
* @param[in] dataLen 		 �������ݵĳ���
* @param[in] cRes 		 ����
*/
BOOL NET_IPC_TransparantSerialControl(long lLoginID,
	int nChannel,
	E_REQ_SERIAL_TYPE eSerialReqType,
	int iParam1,
	int iParam2,
	int iParam3,
	int iParam4,
	BYTE *data,
	int dataLen,
	CBTransData fTransData,
	char cRes);

/**
 * - �ص����������ڻ�ȡ�豸������Ϣ
 * @param[out] alarmInfo ������Ϣ�ṹ��
 */
typedef BOOL (*fAlarmMsgCallBack)(S_ALARM_INFO *alarmInfo);

/**
 * @brief - NET_IPC_AlarmStartListen
 * - ��ʼ��������
 * @param[in] lLoginID �û�ID �ţ�NET_IPC_Login�ķ���ֵ
 * @param[in] DataCallback �ص���������ȡ�豸������Ϣ
 * @return ���ؽ������:
 * - �ɹ�: true
 * - ʧ��: false
 * - ��ȡ���������NET_IPC_GetLastError
 */
BOOL NET_IPC_AlarmStartListen(long lLoginID, fAlarmMsgCallBack DataCallback);

/**
 * @brief - NET_IPC_AlarmStopListen
 * - ֹͣ��������
 * @return ���ؽ������:
 * - �ɹ�: true
 * - ʧ��: false
 * - ��ȡ���������NET_IPC_GetLastError
 */ 
BOOL NET_IPC_AlarmStopListen();

/**
 * - �ص����������ڻ�ȡ����Ƶ����
 * @param[out] lRealHandle ʵʱԤ�����������NET_IPC_StartRealPlay �ķ���ֵ
 * @param[out] iDataType ����Ƶ��������1-��Ƶ����2-��Ƶ����
 * @param[out] pFrameBuf ����Ƶ����
 * @param[out] iFrameSize ����Ƶ���ݳ���
 * @param[out] pUser �û����ݣ�����
 */
typedef void (*fRealDataCallBack)(long lRealHandle, int iDataType, BYTE cRrameType, BYTE cFrameRate, BYTE *pFrameBuf, unsigned int iFrameSize, unsigned long lTimeStamp, void *pUser);

/**
 * @brief - NET_IPC_StartRealPlay
 * - ��ʼʵʱԤ��
 * @param[in] lLoginID �û�ID �ţ�NET_IPC_Login�ķ���ֵ
 * @param[in] sPort �豸�˿ڣ�Ĭ��90
 * @param[in] pRealDataInfo ��������������Ϣ������:
 * - iChannel ͨ���ţ�Ĭ��Ϊ0
 * - iAVType ����Ƶ���ͣ�1-��Ƶ��2-��Ƶ��3-������
 * - iEncodeType ����Ƶ��������1-G711 3-Mjpeg 4-������5-������
 * @param[in] fRealData �ص����������ڻ�ȡ����Ƶ����
 * @param[in] pUserData �û�����
 * @return ����ʵʱԤ���������
 */
long NET_IPC_StartRealPlay(long lLoginID,short sPort,S_REALPLAY_INFO  *pRealDataInfo,fRealDataCallBack fRealData,void *pUserData);

/**
 * @brief - NET_IPC_StopRealPlay
 * - ֹͣʵʱԤ��
 * @param[in] lRealHandle ʵʱԤ�����������NET_IPC_StartRealPlay �ķ���ֵ
 * @return ���ؽ������:
 * - �ɹ�: true
 * - ʧ��: false
 * - ��ȡ���������NET_IPC_GetLastError
 */
BOOL NET_IPC_StopRealPlay(long lRealHandle);

/**
 * @brief - NET_IPC_VoiceFrameInput
 * - �ͻ������豸��������
 * @param[in] lLoginID �û�ID �ţ�NET_IPC_Login�ķ���ֵ
 * @param[in] sPort �����豸�˿ڣ�Ĭ��90
 * @param[in] pRealDataInfo ��������������Ϣ������:
 * - iChannel ͨ���ţ�Ĭ��Ϊ0
 * - iAVType ����Ƶ���ͣ�1-��Ƶ��2-��Ƶ��3-���������˴�Ϊ2
 * - iEncodeType ����Ƶ��������1-G711 3-Mjpeg 4-������5-���������˴�Ĭ��Ϊ1
 * @param[in] pFrameBuf ��������Ƶ֡����
 * @param[in] iFrameSize ��Ƶ֡���ݴ�С���̶�����640
 * @param[in] pUserData �û�����
 * @return ���ؽ������:
 * - �ɹ�: true
 * - ʧ��: false
 * - ��ȡ���������NET_IPC_GetLastError
 */
BOOL NET_IPC_VoiceFrameInput(long lLoginID,short sPort,S_REALPLAY_INFO  *pRealDataInfo,BYTE *pFrameBuf, unsigned int iFrameSize,void *pUserData);

/**
 * @brief - NET_IPC_StopVoiceInput
 * - ֹͣ�ͻ��˵��豸����������
 * @param[in] lLoginID �û�ID �ţ�NET_IPC_Login�ķ���ֵ
 * @return ���ؽ������:
 * - �ɹ�: true
 * - ʧ��: false
 * - ��ȡ���������NET_IPC_GetLastError
 */
BOOL NET_IPC_StopVoiceInput(long lLoginID);

#ifdef __cplusplus
}
#endif //#ifdef __cplusplus

#endif //#ifndef _INF_SDK_NET_H_
