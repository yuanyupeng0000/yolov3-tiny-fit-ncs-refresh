
/*
 * author root
 * date 2016��10��19��
 * File introduction:
 */
#include "client_obj.h"
#include "client_file.h"
#include "file_op.h"
mDetectDeviceConfig device_default_config={
		.deviceID=1,
		.detectport=8888,
		.camnum=1,
		"192.168.1.213",
		"test-cam",
		.cam_info={\
				{1,1,"192.168.1.89"},\
				{1,1,"192.168.1.89"},\
				{1,1,"192.168.1.89"},\
				{1,1,"192.168.1.89"},\
				{1,1,"192.168.1.89"},\
				{1,1,"192.168.1.89"},\
				{1,1,"192.168.1.89"},\
				{1,1,"192.168.1.89"}},
};







mCamAttributes attr_dft={
		.direction=2,
		.camID=22,
		.cammerport=5000,
		.adjustport=5000,
		5000,
		"url:test",
		"admin",
		"admin",
		.cammerIp={'1','9','2','.','1','6','8','.','1','.','8','9'},
		"0.0.0.1",
		"0.0.0.2",
};
mCamDemarcateParam demarcate_dft={
		1,
		2,
		3,
		4,
		5,
		6,
		7
};
mChannelVirtualcoil virtual_coil_dft={
		0,
		0,
		0
};


////////////param////////////
mCamParam camera_default_config={
		.coilnum=2,
		.camattr=attr_dft,
		demarcate_dft,
//		virtual_coil_dft
};


//unsigned int   g_timep[4]; //0 1�����賿 ��1����2   2 3���� �ƻ� ��1����2
//mCamDetectLane g_detectlane;        //��⳵����
mVirtualLaneLine  g_laneline={

};
;       //�궨�������
mStandardPoint g_standpoint[STANDARDVAULEMAX]={

};
//�궨������
mDemDetectArea g_area={

};
// 0  ����Ĳ���,  1�������ϲ���
mDetectParam g_detectparam[ALGMAX]={

};

/////////////////
unsigned int   g_timep[4]={//0 1�����賿 ��1����2   2 3���� �ƻ� ��1����2
		g_timep[0]=0,
		g_timep[0]=0,
		g_timep[0]=0,
		g_timep[0]=0,
};
mChannelCoil virtuallane[DETECTLANENUMMAX]={

};
mCamDetectLane g_detectlane={
};
mCamDetectLane detectlane;        //��⳵����
mVirtualLaneLine  laneline;      //�õ��ĳ�����
mStandardPoint standpoint[STANDARDVAULEMAX];       //�궨�������
mDemDetectArea area;              //�궨������
mDetectParam detectparam[ALGMAX];   // 0  ����Ĳ���,  1�������ϲ���
mCamDetectParam camera_det_default_config={
};

unsigned char coilnum=4;     //ͨ����
mCamAttributes camattr={
		1,
		30,
		5000,
		5000,
		5000,
		"admin",
		"admin",
		"admin",
		"192.168.1.60",
		"192.168.1.60",
		"192.168.1.60",
};
//mCamDemarcateParam camdem;
//mChannelVirtualcoil channelcoil[DETECTLANENUMMAX];
int main()
{
//	load_obj((unsigned char *) &device_default_config,
//			CLASS_mDetectDeviceConfig, 0);

	save_obj((unsigned char *) &device_default_config,
			CLASS_mDetectDeviceConfig, 0);

//	load_obj((unsigned char *) &camera_det_default_config,
//			CLASS_mCamDetectParam, 0);
//	load_obj((unsigned char *) &camera_default_config, CLASS_mCamParam,
//			0);



//	for (int i = 0; i < ACTIVE_CAM_NUM; i++) {
//
//		load_obj((unsigned char *) &camera_det_default_config,
//				CLASS_mCamDetectParam, i);
//		load_obj((unsigned char *) &camera_default_config, CLASS_mCamParam,
//				i);
//
//		save_obj((unsigned char *) &camera_det_default_config,
//				CLASS_mCamDetectParam, i);
//		save_obj((unsigned char *) &camera_default_config, CLASS_mCamParam,
//				i);
//	}
	return 0;
}
