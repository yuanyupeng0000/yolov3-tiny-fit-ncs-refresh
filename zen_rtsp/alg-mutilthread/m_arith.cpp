//#include "stdafx.h"
#include "stdio.h"
#include "stdlib.h" /// div_t I??????
#include <string.h>
#include "m_arith.h"
#include "math.h"
#include "DSPARMProto.h"
#include "Python.h"
#include "pthread.h"
#include <numpy/arrayobject.h>
#include "../common.h"
#include <sys/time.h>
#ifdef DETECT_PLATE//检测车牌
#include "lpr/include/PlateRecognize.h"
#define MAX_PLATE_NUM 10
#endif
#include "intel_dldt.h"
//////////////////////////////////////////////////////////////////////////////////////
//#define DETECT_REAR 
#define	MAX_SPEEDDETECTOR_DOTS	768*576
#define MaxDotsInDetect 768*576
#define MAX_NCS_NUM  50
//#define FULL_COLS  					(720)
//#define FULL_ROWS  					(576)

clock_t current_time;
double timer;
//#define  SAVE_VIDEO
#ifdef SAVE_VIDEO
cv::Mat img;
# define  SAVE_FRAMES  3000
cv::VideoWriter writer("VideoResult.avi", CV_FOURCC('X', 'V', 'I', 'D'), 15, Size(FULL_COLS, FULL_ROWS)); 
#endif
PyObject *pModule;
PyObject *pFunc;
PyGILState_STATE gstate;
int NCS_INDEX[MAX_NCS_NUM] = {0};//计算棒是否在运行
int NCS_NUM = 0;//可用的计算棒数目
//char LABELS[][50] = {"background", "bus", "car", "truck", "bicycle", "motorbike", "person"};
char LABELS[][50] = {"bus","car", "truck", "motorbike", "bicycle","person"};
int init_numpy()
{
	import_array();
	return 1;
}
int initflag = 0;//加载Python，所有相机直执行一次
void py_init()
{
#ifdef USE_PYTHON
	Py_Initialize();
	if ( !Py_IsInitialized() ) {
		printf("init err\n");
	}else{
		printf("init ok\n");
	}
	printf("finding ...\n");
	init_numpy();
	/*pName = PyString_FromString("Test111");


	if(!pName){
	printf("finding err \n");
	}else{
	printf("finding ok \n");
	}*/
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append('./')");
	if(pModule)
		Py_DECREF(pModule);
	pModule = PyImport_ImportModule("mvnc-yolo-tiny");
	if ( !pModule ) {
		printf("can't find .py");
	}else{
		printf("py found\n");
	}
	while(NCS_NUM <= 0)
	{
		PyObject* result;
		//PyObject* pFunc;
		//pFunc = PyObject_GetAttrString(pModule, "release");//释放计算棒
		pFunc = PyObject_GetAttrString(pModule, "init");
		result = PyObject_CallObject(pFunc, NULL);////初始化计算棒,得到读取计算棒的个数
		PyArg_Parse(result, "i", &NCS_NUM);
		if(NCS_NUM <= 0)
		{
			printf("no device open,continuing load NCS device\n");
		}
		if(result)
			Py_DECREF(result);
		if(pFunc)
			Py_DECREF(pFunc);
	}
	printf("Load NCS devices ok！ NCS num =%d\n",NCS_NUM);
	pFunc = PyObject_GetAttrString(pModule, "process1");
	PyEval_InitThreads(); 
	PyEval_ReleaseThread(PyThreadState_Get()); 
#else
    printf("[ INFO ] Intel dldt init \n");
    NCS_NUM = intel_dldt_init("FP16/vpu_config.ini");
    printf("[ INFO ] Intel dldt init end. NCS_NUM = %d\n", NCS_NUM);
#endif
	memset(NCS_INDEX, 0, MAX_NCS_NUM * sizeof(int));
}
void py_free()
{
	printf("py free\n");
	/*if(pName)
		Py_DECREF(pName);
	if(pModule)
		Py_DECREF(pModule);
	if(pFunc)
		Py_DECREF(pFunc);*/
	// 关闭Python
	//Py_Finalize();
}

int alg_mem_malloc(m_args *p)
{
	//printf("malloc.......\n");
	int ret = -1;
	int size;
	int i = 0;
	//输出内存分配

	//	ALGPARAMS *Params;
	p->p_outbuf = NULL;
	p->p_outbuf = (OUTBUF*) malloc(sizeof(OUTBUF));
	if (p->p_outbuf == NULL) {
		printf("alg malloc err\n");
	}
	memset(p->p_outbuf, 0, sizeof(OUTBUF));

	//参数配置内存分配
	p->pParams = NULL;
	p->pParams = (ALGPARAMS*) malloc(sizeof(ALGPARAMS));
	if (p->pParams == NULL) {
		printf("alg malloc err\n");
	}
	memset(p->pParams, 0, sizeof(ALGPARAMS));
	p->pParams->puPointNewImage = NULL;

	p->pParams->puPointNewImage = (Uint8*) malloc(
		DETECTRECT_WIDTH_MAX * DETECTRECT_HEIGHT_MAX * 8 * sizeof(Uint8));
	if (p->pParams->puPointNewImage == NULL) {
		printf("alg malloc err\n");
	}

	memset(p->pParams->puPointNewImage, 0,
		DETECTRECT_WIDTH_MAX * DETECTRECT_HEIGHT_MAX * 8 * sizeof(Uint8));
	memset(p->pParams->puPointNewImage, 0, 1000);

	//配置内存分配
	p->pCfgs = NULL;
	p->pCfgs = (ALGCFGS*) malloc(sizeof(ALGCFGS));
	if (p->pCfgs == NULL) {
		printf("alg malloc err\n");
	}else{
		printf("alg malloc ok \n");
	}

	memset(p->pCfgs, 0, sizeof(ALGCFGS));
	/*p->pCfgs->CameraLocalPara.CameraCfgEntry = NULL;
	size = DETECTRECT_WIDTH_MAX * DETECTRECT_HEIGHT_MAX * sizeof(Uint32);
	size = sizeof(CAMERA_STRUCT) + MAX_LANE * sizeof(SPEED_DETECT_STRUCT) + size;
	p->pCfgs->CameraLocalPara.CameraCfgEntry = (Uint8*) malloc(size);
	if (p->pCfgs->CameraLocalPara.CameraCfgEntry == NULL) {
		printf("alg malloc err\n");
	}

	memset(p->pCfgs->CameraLocalPara.CameraCfgEntry, 0, size);
	p->pCfgs->CameraLocalPara.ImageStorageEntry = NULL;
	size = DETECTRECT_WIDTH_MAX * DETECTRECT_HEIGHT_MAX * 8 * sizeof(Uint16);
	p->pCfgs->CameraLocalPara.ImageStorageEntry = (Uint8*) malloc(size);
	if (p->pCfgs->CameraLocalPara.ImageStorageEntry == NULL) {
		printf("alg malloc err\n");
	}

	memset(p->pCfgs->CameraLocalPara.ImageStorageEntry, 0, size);*/
	if(initflag == 0)
		py_init();
	initflag++;
	//给相机分配计算棒
	p->pCfgs->NCS_ID = -1;
	for(i = 0; i < NCS_NUM; i++)
	{
		if(NCS_INDEX[i] == 0)
		{
			p->pCfgs->NCS_ID = i;
			NCS_INDEX[i] = i + 1;
			break;
		}
	}
	if(i == NCS_NUM && p->pCfgs->NCS_ID < 0)
	{
		printf("no device usable\n");
	}

	printf("ncs_id =%d\n",p->pCfgs->NCS_ID);
#ifdef DETECT_PLATE//加载车牌识别网络
	p->pCfgs->plate_flag = LoadPlateNet();
	if(p->pCfgs->plate_flag < 0)
		printf("no load plate recognize net\n");
	printf("load plate recognize\n");
#endif
	//printf("(pCfgs malloc %x,%x\n",p->pCfgs,pthread_self());fflush(NULL);
	//printf("(pCfgs malloc %x\n",p->pCfgs->CameraCfg);
	return 0;
}

int alg_mem_free(m_args *arg_arg)
{
	//printf("free.......\n");
	if(arg_arg->pCfgs->NCS_ID >= 0)
	{
		NCS_INDEX[arg_arg->pCfgs->NCS_ID] = 0;//设置此计算棒不能运行
	}
#ifdef DETECT_PLATE//释放车牌识别网络
	if(arg_arg->pCfgs->plate_flag >= 0)
	{
		FreePlateNet(arg_arg->pCfgs->plate_flag);
	}
#endif
	//py_free();
	if (arg_arg->pCfgs) {
		/*if (arg_arg->pCfgs->CameraLocalPara.ImageStorageEntry) {
			free(arg_arg->pCfgs->CameraLocalPara.ImageStorageEntry);
			arg_arg->pCfgs->CameraLocalPara.ImageStorageEntry = NULL;
		}

		if (arg_arg->pCfgs->CameraLocalPara.CameraCfgEntry) {
			free(arg_arg->pCfgs->CameraLocalPara.CameraCfgEntry);
			arg_arg->pCfgs->CameraLocalPara.CameraCfgEntry = NULL;
		}*/

		free(arg_arg->pCfgs);
		arg_arg->pCfgs = NULL;
	}
	if ( arg_arg->pParams) {
		if (arg_arg->pParams->puPointNewImage) {
			free(arg_arg->pParams->puPointNewImage);
			arg_arg->pParams->puPointNewImage = NULL;
		}
		free(arg_arg->pParams);
		arg_arg->pParams = NULL;
	}
	if (arg_arg->p_outbuf) {
		free(arg_arg->p_outbuf);
		arg_arg->p_outbuf = NULL;
	}
	return 0;
}

#define max(a,b) (((a)>(b)) ? (a):(b))
#define min(a,b) (((a)>(b)) ? (b):(a))
//校正区域四个点的坐标顺序，按顺时针 0 1 2 3
void CorrectRegionPoint(CPoint* ptCorner)
{
	CPoint temp;
	if(ptCorner[0].y > ptCorner[3].y)//左上的y值大于左下
	{
		temp = ptCorner[0];
		ptCorner[0] = ptCorner[3];
		ptCorner[3] = temp;
	}
	if(ptCorner[0].y > ptCorner[2].y)//左上的y值大于右下
	{
		temp = ptCorner[0];
		ptCorner[0] = ptCorner[2];
		ptCorner[2] = temp;
	}
	if(ptCorner[1].y > ptCorner[3].y)//右上的y值大于左下
	{
		temp = ptCorner[1];
		ptCorner[1] = ptCorner[3];
		ptCorner[3] = temp;
	}
	if(ptCorner[1].y > ptCorner[2].y)//右上的y值大于右下
	{
		temp = ptCorner[1];
		ptCorner[1] = ptCorner[2];
		ptCorner[2] = temp;
	}
	if(ptCorner[0].x > ptCorner[1].x)//左上的x值大于右上
	{
		temp = ptCorner[0];
		ptCorner[0] = ptCorner[1];
		ptCorner[1] = temp;
	}
	if(ptCorner[3].x > ptCorner[2].x)//左下的x值大于右下
	{
		temp = ptCorner[3];
		ptCorner[3] = ptCorner[2];
		ptCorner[2] = temp;
	}

}
//判断一个点是否在四边形里
bool isPointInRect(CPoint pt, CPoint mLBPoint, CPoint mLTPoint, CPoint mRTPoint, CPoint mRBPoint) 
{
	int a = (mLTPoint.x - mLBPoint.x) * (pt.y - mLBPoint.y) - (mLTPoint.y - mLBPoint.y) * (pt.x - mLBPoint.x);
	int b = (mRTPoint.x - mLTPoint.x) * (pt.y - mLTPoint.y) - (mRTPoint.y - mLTPoint.y) * (pt.x - mLTPoint.x);
	int c = (mRBPoint.x - mRTPoint.x) * (pt.y - mRTPoint.y) - (mRBPoint.y - mRTPoint.y) * (pt.x - mRTPoint.x);
	int d = (mLBPoint.x - mRBPoint.x) * (pt.y - mRBPoint.y) - (mLBPoint.y - mRBPoint.y) * (pt.x - mRBPoint.x);
	if((a > 0 && b > 0 && c > 0 && d > 0) || (a < 0 && b < 0 && c < 0 && d < 0)) 
	{
		return TRUE;
	}

	return FALSE; 
}
//设置pParams->MaskDetectImage为行人检测区域
bool MaskDetectImage(ALGCFGS *pCfgs, ALGPARAMS *pParams, int imgW, int imgH)
{
	Int32	i, j, k;
	CPoint	ptCorner[4];
	Uint8* p;
	CPoint pt;
	//标记检测区域,如果图像大小大于640*480，resize到640*480
	memset(pParams->MaskDetectImage, 0, pCfgs->m_iWidth * pCfgs->m_iHeight);
	//传入行人检测区域
	for(i = 0; i < pCfgs->uDetectRegionNum; i++)	
	{
		ptCorner[0].x = pCfgs->DownSpeedCfg.PersonDetectArea.area[i].realcoordinate[0].x;
		ptCorner[0].y = pCfgs->DownSpeedCfg.PersonDetectArea.area[i].realcoordinate[0].y;
		ptCorner[1].x = pCfgs->DownSpeedCfg.PersonDetectArea.area[i].realcoordinate[1].x;
		ptCorner[1].y = pCfgs->DownSpeedCfg.PersonDetectArea.area[i].realcoordinate[1].y;
		ptCorner[2].x = pCfgs->DownSpeedCfg.PersonDetectArea.area[i].realcoordinate[2].x;
		ptCorner[2].y = pCfgs->DownSpeedCfg.PersonDetectArea.area[i].realcoordinate[2].y;
		ptCorner[3].x = pCfgs->DownSpeedCfg.PersonDetectArea.area[i].realcoordinate[3].x;
		ptCorner[3].y = pCfgs->DownSpeedCfg.PersonDetectArea.area[i].realcoordinate[3].y;
		printf("person region = %d,[%d,%d,%d,%d,%d,%d,%d,%d]\n",i,ptCorner[0].x,ptCorner[0].y,ptCorner[1].x,ptCorner[1].y,ptCorner[2].x,ptCorner[2].y,ptCorner[3].x,ptCorner[3].y);
		/*//采用标定区域作为检测区域
		for(j = 0; j < 4; j++)
		{
			ptCorner[j].x = pCfgs->calibration_point[j][0];
			ptCorner[j].y = pCfgs->calibration_point[j][1];
		}*/
		//按照顺时针方向矫正4点顺序
		CorrectRegionPoint(ptCorner);
		//车道坐标不在检测区域内，返回
		for(j = 0; j < 4; j++)
		{
			/*if(ptCorner[j].x < 0 || ptCorner[j].x > imgW || ptCorner[j].y < 0 || ptCorner[j].y > imgH)
			{
				printf("detect person region Point err\n");
				return FALSE;
			}*/
			ptCorner[j].x = ptCorner[j].x * pCfgs->m_iWidth /imgW;
			ptCorner[j].y = ptCorner[j].y * pCfgs->m_iHeight /imgH;
		}
		//对pParams->MaskDetectImage进行初始化代表不同的检测区域
		for(j = 0; j < pCfgs->m_iHeight; j++)
		{
			p = pParams->MaskDetectImage + j * pCfgs->m_iWidth;
			for(k = 0; k < pCfgs->m_iWidth; k++)
			{
				pt.x = k;
				pt.y = j;
				if(isPointInRect(pt, ptCorner[3], ptCorner[0], ptCorner[1], ptCorner[2]))
				{
					p[k] = i + 1;
					//p[k] = 255;
				}
			}
		}
	}
	/*IplImage* mask = cvCreateImage(cvSize(pCfgs->m_iWidth, pCfgs->m_iHeight), IPL_DEPTH_8U, 1);
	memcpy(mask->imageData, pParams->MaskDetectImage, pCfgs->m_iWidth * pCfgs->m_iHeight);
	cvSaveImage("mask.jpg", mask, 0);
	cvReleaseImage(&mask);*/

	return	TRUE;

}

//设置pParams->MaskLaneImage为车道区域
bool MaskLaneImage(ALGCFGS *pCfgs, ALGPARAMS *pParams, int imgW, int imgH)
{
	Int32	i, j, k;
	CPoint	ptCorner[4];
	Uint8* p;
	CPoint pt;
	//标记检测区域,如果图像大小大于640*480，resize到640*480
	memset(pParams->MaskLaneImage, 0, pCfgs->m_iWidth * pCfgs->m_iHeight);
	//车道区域
	for(i = 0; i < pCfgs->CameraCfg.LaneAmount; i++)	
	{
		memcpy( (void*)ptCorner, (void*)pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion, 4 * sizeof(CPoint) );
		printf("lane region = %d,[%d,%d,%d,%d,%d,%d,%d,%d]\n",i,ptCorner[0].x,ptCorner[0].y,ptCorner[1].x,ptCorner[1].y,ptCorner[2].x,ptCorner[2].y,ptCorner[3].x,ptCorner[3].y);
		//按照顺时针方向矫正4点顺序
		CorrectRegionPoint(ptCorner);
		//车道坐标不在检测区域内，返回
		/*for(j = 0; j < 4; j++)
		{
			if(ptCorner[j].x < 0 || ptCorner[j].x > imgW || ptCorner[j].y < 0 || ptCorner[j].y > imgH)
			{
				printf("detect lane region Point err\n");
				return FALSE;
			}
		}*/
		//将坐标限制在[0 pCfgs->m_iWidth - 1]和[0 pCfgs->m_iHeight-1]
		for(j = 0; j < 4; j++)
		{
			ptCorner[j].x = ptCorner[j].x * pCfgs->m_iWidth / imgW;
			ptCorner[j].y = ptCorner[j].y * pCfgs->m_iHeight / imgH;
		}
		//对pParams->MaskLaneImage进行初始化代表不同的车道区域
		for(j = 0; j < pCfgs->m_iHeight; j++)
		{
			p = pParams->MaskLaneImage + j * pCfgs->m_iWidth;
			for(k = 0; k < pCfgs->m_iWidth; k++)
			{
				pt.x = k;
				pt.y = j;
				if(isPointInRect(pt, ptCorner[3], ptCorner[0], ptCorner[1], ptCorner[2]))
				{
					p[k] = i + 1;
				}
			}
		}
	}
	/*IplImage* mask = cvCreateImage(cvSize(pCfgs->m_iWidth, pCfgs->m_iHeight), IPL_DEPTH_8U, 1);
	memcpy(mask->imageData, pParams->MaskLaneImage, pCfgs->m_iWidth * pCfgs->m_iHeight);
	cvSaveImage("masklane.jpg", mask, 0);
	cvReleaseImage(&mask);*/

	return	TRUE;

}
//对图像进行标定
void get_calibration_data(ALGCFGS *pCfgs, int imgW, int imgH)
{
	int i = 0, j = 0, k = 0;
	int min_value = 1000;
	int idx = 0;
	CPoint pt1[2];
	//图像标定
	//if((pCfgs->calibration_point[2][1] > pCfgs->calibration_point[0][1]) && (pCfgs->calibration_point[3][1] > pCfgs->calibration_point[1][1]))
	camera_calibration(pCfgs->base_line, pCfgs->base_length, pCfgs->calibration_point, pCfgs->near_point_length, pCfgs->CameraCfg.LaneAmount, pCfgs, imgW, imgH);
	//得到远近线圈的实际距离
	for(i = 0; i < pCfgs->CameraCfg.LaneAmount; i++)
	{
		pt1[0].y = (pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[0].y + pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[1].y) / 2;
		pt1[1].y = (pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[2].y + pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[3].y) / 2;
		pCfgs->uActualTailLength[i] = int(abs(pCfgs->actual_distance[i][pt1[0].y] - pCfgs->actual_distance[i][pt1[1].y]) + 0.5);
		pt1[0].y = (pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[0].y + pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[1].y) / 2;
		pt1[1].y = (pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[2].y + pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[3].y) / 2;
		pCfgs->uActualDetectLength[i] = int(abs(pCfgs->actual_distance[i][pt1[0].y] - pCfgs->actual_distance[i][pt1[1].y]) + 0.5);
	}
	printf("coil actual length = %d,%d\n",pCfgs->uActualTailLength[0], pCfgs->uActualDetectLength[1]);
	//得到刻度线点
	for(i = 0; i < 20; i++)
	{
		pCfgs->degreepoint[i][0] = 0;
		pCfgs->degreepoint[i][1] = 0;
	}
	j = ((int)(pCfgs->near_point_length + 9) / 10 + 1) * 10;
	k = 0;
	for(i = imgH - 1; i >= 0; i--)
	{
		if(abs(pCfgs->actual_degree_length[i] - j) < min_value)
		{
			min_value = abs(pCfgs->actual_degree_length[i] - j);
			idx = i;
		}
		if((int)(pCfgs->actual_degree_length[i] + 0.5) == j)
		{
			pCfgs->degreepoint[k][0] = idx;
			pCfgs->degreepoint[k][1] = j;
			//printf("degree_y=%d,len=%d\n", pCfgs->degreepoint[k][0], pCfgs->degreepoint[k][1]);
			min_value = 1000;
			j = j + 10;
			k++;


		}
		if(k == 20)
		{
			break;
		} 
	}
	//pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uDegreeLength=pCfgs->actual_degree_length;
}
//设置检测线
int SetLine(ALGCFGS* pCfgs, CPoint ptDetLine[][2])
{
	int i = 0;
	bool bDetPersonFlow = FALSE;
	//检测线未设置或者错误
	for( i = 0; i < pCfgs->uDetectRegionNum; i++)
	{
		if((ptDetLine[i][0].x <= 0 && ptDetLine[i][0].y <= 0 && ptDetLine[i][1].x <= 0 && ptDetLine[i][1].y <= 0) || (ptDetLine[i][0].x == ptDetLine[i][1].x  && ptDetLine[i][0].y == ptDetLine[i][1].y))
		{
			printf("detect line error\n");
			pCfgs->detLineParm[i].k = 0;
			pCfgs->detLineParm[i].b = 0;
			pCfgs->detLineParm[i].line_vertical = 0;
			continue;
		}
		printf("line %d:[%d,%d],[%d,%d]\n",ptDetLine[i][0].x,ptDetLine[i][0].y,ptDetLine[i][1].x,ptDetLine[i][1].y);
		pCfgs->detLineParm[i].pt[0] = ptDetLine[i][0];
		pCfgs->detLineParm[i].pt[1] = ptDetLine[i][1];
		pCfgs->detLineParm[i].detLeft = min(ptDetLine[i][0].x, ptDetLine[i][1].x);
		pCfgs->detLineParm[i].detTop = min(ptDetLine[i][0].y, ptDetLine[i][1].y);
		pCfgs->detLineParm[i].detRight = max(ptDetLine[i][0].x, ptDetLine[i][1].x);
		pCfgs->detLineParm[i].detBottom = max(ptDetLine[i][0].y, ptDetLine[i][1].y);

		//计算检测线的斜率和截距
		if(abs(ptDetLine[i][0].x - ptDetLine[i][1].x) < 5)//检测线垂直
		{
			pCfgs->detLineParm[i].k = 0;
			pCfgs->detLineParm[i].b = (ptDetLine[i][0].y + ptDetLine[i][1].y) / 2; 
			pCfgs->detLineParm[i].line_vertical = 1;
		}
		else
		{
			pCfgs->detLineParm[i].k = (float)(ptDetLine[i][1].y - ptDetLine[i][0].y) / (float)(ptDetLine[i][1].x - ptDetLine[i][0].x);
			pCfgs->detLineParm[i].b = ptDetLine[i][1].y - pCfgs->detLineParm[i].k * ptDetLine[i][1].x;
			pCfgs->detLineParm[i].line_vertical = 0;
		}
		bDetPersonFlow = TRUE;
	}
	//检测线设置成功，检测行人方向流量数
	pCfgs->bDetPersonFlow = bDetPersonFlow;
	return 1;
}
//分方向得到检测线处的行人数
void get_object_num(ALGCFGS* pCfgs, int target_idx, int region_idx)
{
	int x = 0, y = 0;
	int direction = 0;
	x = pCfgs->objPerson[target_idx].box.x + pCfgs->objPerson[target_idx].box.width / 2 ;
	y = pCfgs->objPerson[target_idx].box.y + pCfgs->objPerson[target_idx].box.height / 2 ;
	//得到行人方向，根据检测线端点的顺序确定正反方向
	if(pCfgs->detLineParm[region_idx].line_vertical)
	{
		if(x > pCfgs->detLineParm[region_idx].pt[0].x)
		{
			//direction = (pCfgs->detLineParm[region_idx].pt[1].y > pCfgs->detLineParm[region_idx].pt[0].y)? 0 : 1;
			direction = 0;
		}
		else
		{
			//direction = (pCfgs->detLineParm[region_idx].pt[1].y > pCfgs->detLineParm[region_idx].pt[0].y)? 1 : 0;
			direction = 1;
		}
	}
	else
	{
		if(pCfgs->detLineParm[region_idx].k * x + pCfgs->detLineParm[region_idx].b - y > 0)
		{
			//direction = (pCfgs->detLineParm[region_idx].pt[1].x > pCfgs->detLineParm[region_idx].pt[0].x)? 0 : 1;
			direction = 1;
		}
		else
		{
			//direction = (pCfgs->detLineParm[region_idx].pt[1].x > pCfgs->detLineParm[region_idx].pt[0].x)? 1 : 0;
			direction = 0;
		}
	}
	//分方向统计
	if(strcmp(pCfgs->objPerson[target_idx].names, "person") == 0)
	{
		pCfgs->uPersonDirNum[region_idx][direction]++;
	}
}
//初始化参数
bool ArithInit(Uint16 ChNum, CFGINFOHEADER *pCfgHeader, SPEEDCFGSEG *pDetectCfgSeg, ALGCFGS *pCfgs, ALGPARAMS *pParams)//,CPoint m_ptend[],CPoint LineUp[]
{
	bool 	bValidCfg;

	if(TRUE == CfgStructParse(ChNum, pCfgHeader, pDetectCfgSeg, pCfgs, pParams))//,m_ptend,LineUp
	{
		bValidCfg = TRUE;
	}
	else
	{
		bValidCfg = FALSE;
	}
	if(bValidCfg == TRUE)
	{
		if( ArithVariableInit(ChNum, pCfgs, pParams) == TRUE )
		{
			return TRUE;
		}
		else
		{
			return FALSE;
		}

	}
	else
	{
		return FALSE;
	}
}

//得到检测框是否在检测区域
int RectInRegion(unsigned char* maskImage, int width, int height, CRect rct, int idx)
{
	int InRegionRatio = 0;
	int i = 0, j = 0;
	int num = 0, totalnum = 0;
	int val = 0;
	unsigned char* p;
	for(i = rct.y; i < (rct.y + rct.height); i += 2)
	{
		p = maskImage + i * width;
		for(j = rct.x; j < (rct.x + rct.width); j += 2)
		{
			val = *(p + j);
			//if(val & (1 << idx))
			if(val == (idx + 1))
			{
				num++;
			}
			totalnum++;
		}
	}
	InRegionRatio = (float)(num * 100) / (float) totalnum;
	return InRegionRatio;
}

//配置参数
bool CfgStructParse(Uint16 ChNum, CFGINFOHEADER *pCfgHeader, SPEEDCFGSEG *pDetectCfgSeg, ALGCFGS *pCfgs, ALGPARAMS *pParams)//,CPoint m_ptend[],CPoint LineUp[]
{
	Int32	i, j, k, idx;
	float min_value = 1000;
	ZENITH_SPEEDDETECTOR 		*pDownSpeedDetect = NULL;
	bool 	bDetectBackRun[8];
	CPoint	ptFlowCorner[4];//流量区域坐标点
	CPoint	ptMiddleCoil[4];//占有线圈坐标点 
	CPoint	ptFrontCoil[4];//占位线圈坐标点
	CPoint  ptLaneRegion[4];//车道区域坐标点
	CPoint  pt1[MAX_REGION_NUM][2];

	pDownSpeedDetect = (ZENITH_SPEEDDETECTOR*)pDetectCfgSeg->uSegData;

	//加载参数
	pCfgs->CameraCfg.LaneAmount = pDownSpeedDetect->uLaneTotalNum;
	pCfgs->bAuto = pDownSpeedDetect->uEnvironment; //TRUE;
	pCfgs->CameraCfg.uEnvironmentStatus = pDownSpeedDetect->uEnvironmentStatus;
	pCfgs->CameraCfg.uDayNightJudgeMinContiuFrame = pDownSpeedDetect->uDayNightJudgeMinContiuFrame; //no used
	pCfgs->CameraCfg.uGetBackMinContiuFrame = pDownSpeedDetect->uComprehensiveSens;  //no used
	pCfgs->CameraCfg.uVehicleHeadMinRow = pDownSpeedDetect->uDetectSens1;
	pCfgs->CameraCfg.uInThreshold = pDownSpeedDetect->uDetectSens2; 
	pCfgs->CameraCfg.uSquareThreshold = pDownSpeedDetect->uStatisticsSens1;
	pCfgs->CameraCfg.guSubImageThreshold = pDownSpeedDetect->uStatisticsSens2;
	pCfgs->CameraCfg.uSobelThreshold = pDownSpeedDetect->uSobelThreshold;

	pCfgs->CameraCfg.bCameraWorkInCross = pCfgHeader->uDetectPosition;  
	pCfgs->CameraLocalPara.bNormalDetectEnable = pCfgHeader->uDetectFuncs[0]; 
	////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////
	for(i = 0; i < pCfgs->CameraCfg.LaneAmount; i++) //
	{
		if(2 == pDownSpeedDetect->SpeedEachLane[i].uDetectDerection)
		{
			bDetectBackRun[i] = FALSE;
		}
		else if(1 == pDownSpeedDetect->SpeedEachLane[i].uDetectDerection)
		{
			bDetectBackRun[i] = TRUE;
		}

	}
	//得到每个车道参数，并对图像区域进行矫正
	for(i=0; i<pCfgs->CameraCfg.LaneAmount; i++)
	{
		memcpy( (void*)ptFlowCorner, (void*)pDownSpeedDetect->SpeedEachLane[i].RearCoil, 4 * sizeof(CPoint));
		memcpy( (void*)ptMiddleCoil, (void*)pDownSpeedDetect->SpeedEachLane[i].MiddleCoil, 4 * sizeof(CPoint));
		memcpy( (void*)ptFrontCoil, (void*)pDownSpeedDetect->SpeedEachLane[i].FrontCoil, 4 * sizeof(CPoint));
		memcpy( (void*)ptLaneRegion, (void*)pDownSpeedDetect->SpeedEachLane[i].LaneRegion, 4 * sizeof(CPoint));

		CorrectRegionPoint(ptFlowCorner);//矫正流量区域
		CorrectRegionPoint(ptMiddleCoil);//矫正占有区域
		CorrectRegionPoint(ptFrontCoil);//矫正占位区域
		CorrectRegionPoint(ptLaneRegion);//矫正车道区域
				
		memcpy( (void*)pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil, (void*)ptFlowCorner, 4 * sizeof(CPoint) );
		memcpy( (void*)pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil, (void*)ptMiddleCoil, 4 * sizeof(CPoint) );
		memcpy( (void*)pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil, (void*)ptFrontCoil, 4 * sizeof(CPoint) );
		memcpy( (void*)pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion, (void*)ptLaneRegion, 4 * sizeof(CPoint) );

		printf("lane region = [%d,%d,%d,%d,%d,%d,%d,%d]\n",pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[0].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[0].y,pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[1].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[1].y,pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[2].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[2].y,pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[3].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[3].y);
		printf("front region = [%d,%d,%d,%d,%d,%d,%d,%d]\n",pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[0].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[0].y,pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[1].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[1].y,pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[2].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[2].y,pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[3].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[3].y);
		printf("middle region = [%d,%d,%d,%d,%d,%d,%d,%d]\n",pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[0].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[0].y,pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[1].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[1].y,pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[2].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[2].y,pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[3].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[3].y);
		printf("rear region = [%d,%d,%d,%d,%d,%d,%d,%d]\n",pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[0].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[0].y,pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[1].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[1].y,pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[2].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[2].y,pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[3].x,pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[3].y);
	}

	//得到4个标定点和基准线端点
	for(i = 0; i < 4; i++)
	{
		pCfgs->calibration_point[i][0] = pDownSpeedDetect->ptimage[i].x;
		pCfgs->calibration_point[i][1] = pDownSpeedDetect->ptimage[i].y;

	}
	for(i = 4; i < 6; i++)
	{
		pCfgs->base_line[i - 4][0] = pDownSpeedDetect->ptimage[i].x;
		pCfgs->base_line[i - 4][1] = pDownSpeedDetect->ptimage[i].y;
	}
	pCfgs->base_length = pDownSpeedDetect->base_length;
	pCfgs->near_point_length = pDownSpeedDetect->near_point_length;
	printf("calibration point = [%d,%d],[%d,%d],[%d,%d],[%d,%d]\n",pCfgs->calibration_point[0][0],pCfgs->calibration_point[0][1],pCfgs->calibration_point[1][0],pCfgs->calibration_point[1][1],pCfgs->calibration_point[2][0],pCfgs->calibration_point[2][1],pCfgs->calibration_point[3][0],pCfgs->calibration_point[3][1]);
	printf("base line = [%d,%d],[%d,%d]\n",pCfgs->base_line[0][0],pCfgs->base_line[0][1],pCfgs->base_line[1][0],pCfgs->base_line[1][1]);
	printf("base length = %f,near_point_length =%f\n",pCfgs->base_length,pCfgs->near_point_length);

	//加载行人检测参数
	pCfgs->DownSpeedCfg.PersonDetectArea = pDownSpeedDetect->PersonDetectArea;//行人检测区域
	pCfgs->uDetectRegionNum = pCfgs->DownSpeedCfg.PersonDetectArea.num;//行人区域数
	//行人检测线
	for(i = 0; i < pCfgs->uDetectRegionNum; i++)
	{
		pt1[i][0].x = pDownSpeedDetect->PersonDetectArea.area[i].detectline[0].x;
		pt1[i][0].y = pDownSpeedDetect->PersonDetectArea.area[i].detectline[0].y;
		pt1[i][1].x = pDownSpeedDetect->PersonDetectArea.area[i].detectline[1].x;
		pt1[i][1].y = pDownSpeedDetect->PersonDetectArea.area[i].detectline[1].y;
		memset(pCfgs->uPersonDirNum[i], 0, MAX_DIRECTION_NUM * sizeof(Uint16));//行人方向数
	}
	//求行人检测线的斜率和截距
	SetLine(pCfgs, pt1);
	pCfgs->person_id = 1;//行人目标ID
	pCfgs->objPerson_size = 0;//行人目标数
	memset(pCfgs->uRegionPersonNum, 0, MAX_REGION_NUM * sizeof(Uint16));//区域行人数
 
	////////////////////////////////////////////////////detect params初始化参数
	pCfgs->target_id = 1;
	pCfgs->targets_size = 0;
	pCfgs->classes = sizeof(LABELS)/sizeof(LABELS[0]);
	memset(pCfgs->targets, 0, MAX_TARGET_NUM * sizeof(CTarget));
	memset(pCfgs->detClasses, 0 , MAX_CLASSES * sizeof(CDetBox));
	for( i = 0; i < MAX_LANE; i++)
	{
		memset(pCfgs->detBoxes, 0 , MAX_LANE_TARGET_NUM * sizeof(Rect));
	}
	memset(pCfgs->detNum, 0, MAX_LANE * sizeof(Uint16));
	//memset(pCfgs->uStatPersonNum, 0, 4 * sizeof(Uint16));
	memset(pCfgs->detTargets, 0, MAX_TARGET_NUM * sizeof(CTarget));
	pCfgs->detTarget_id = 1;
	pCfgs->detTargets_size = 0;
	return	TRUE;

}
//设置检测参数
bool ArithVariableInit(Uint16 ChNum, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	Int32	i;

	pCfgs->gThisFrameTime = 0; 
	//分配内存
	pParams->MaskLaneImage =(Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX * DETECTRECT_HEIGHT_MAX; 
	pParams->CurrQueueImage = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX * DETECTRECT_HEIGHT_MAX * 2;
	pParams->PreQueueImage = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX * DETECTRECT_HEIGHT_MAX * 3;
	pParams->BackQueueImage = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX * DETECTRECT_HEIGHT_MAX * 4;
	pParams->PrePreQueueImage = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX * DETECTRECT_HEIGHT_MAX * 5;
	pParams->PrePrePreQueueImage = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX * DETECTRECT_HEIGHT_MAX * 6;
	pParams->MaskDetectImage = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX * DETECTRECT_HEIGHT_MAX * 7;
	//初始化参数
	for(i=0; i<MAX_LANE; i++)
	{
		pCfgs->uDetectVehicleSum[i] = 0;
		memset(pCfgs->uStatVehicleSum[i], 0, 4 * sizeof(Uint16));
		memset(pCfgs->uStatQuePos[i], 0, 6 * sizeof(Uint16));
		pCfgs->uDetectVehicleFrameNum[i] = 0;
		pCfgs->uRearIntervalNum[i] = 0;//后线圈两目标之间间隔
		pCfgs->existFrameNum[i][0] = 0;
		pCfgs->existFrameNum[i][1] = 0;
	}
	////20131219	
	pParams->NormalDetectCfg.cSpeedDesc.getQueback_flag=TRUE;
	pCfgs->bMaskDetectImage = FALSE;//未设置检测区域掩模图像
	pCfgs->bMaskLaneImage = FALSE;//未设置车道掩模图像
	pCfgs->bCalibrationImage = FALSE;//未进行标定

	return TRUE;  //ADDED BY DAVID 20131023

}
//根据坐标位置，保存图像数据到CurrQueueImage
void StoreCurImage(Uint8 *inBuf, ALGCFGS *pCfgs, ALGPARAMS	*pParams)
{
	Uint32	i,j;
	Uint32	QElements;
	Uint32 	index;
	Uint32	*pCoordinatePointer;
	struct 	cSpeedDetectStruct *pSpeedDetect = NULL;

	for( i = 0; i < pCfgs->CameraCfg.LaneAmount; i++ )
	{
		pSpeedDetect = pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr;
		QElements = pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->QueDetectDots;
		pCoordinatePointer = pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->CoordinatePointerQ;
		
		for(j = 0; j < QElements; j++)
			{
				index = *(pCoordinatePointer + j);
				/*index = index * 2 + 1;*/
				*((Uint8*)(pSpeedDetect->CurrQueueImage) + j) = *(inBuf + index);//txl,0701
			}
			//
	}
	return;
}
//采用计算棒进行检测
Uint16 ArithDetect(ALGCFGS* pCfgs, Mat img, int* rst)
{
	struct timeval start_time, end_time1,end_time2;
	gettimeofday( &start_time, NULL );
	if(pCfgs->NCS_ID  < 0)
	{
		printf("no device\n");
		return 0;
	}
	int width = img.cols;
	int height = img.rows;
	int size = 0;
	Uint16 nboxes = 0;
 #ifdef USE_PYTHON
	//传数据给python,进行检测
	unsigned char* imagedata = (unsigned char *)malloc(width * height * 3);
	memcpy(imagedata, img.data, width * height * 3);
	npy_intp Dims[3]= { height, width, 3}; //给定维度信息
	gettimeofday( &end_time1, NULL );
	gstate = PyGILState_Ensure();   //如果没有GIL，则申请获取GIL
	Py_BEGIN_ALLOW_THREADS;
	Py_BLOCK_THREADS;
	PyObject* PyListRGB = PyArray_SimpleNewFromData(3, Dims, NPY_UBYTE, imagedata);
	PyObject* ArgList = PyTuple_New(4);
	PyTuple_SetItem(ArgList, 0, PyListRGB);//将PyList对象放入PyTuple对象中
	PyTuple_SetItem(ArgList, 1, Py_BuildValue("i", width));
	PyTuple_SetItem(ArgList, 2, Py_BuildValue("i", height));
	PyTuple_SetItem(ArgList, 3, Py_BuildValue("i", pCfgs->NCS_ID));
	/*int resize_width = 300;
	int resize_height = 300;
	Mat resizeImage;
	resize(BGRImage, resizeImage, Size(300,300));
	//传数据给python,进行检测
	unsigned char* imagedata = (unsigned char *)malloc(resize_width * resize_height * 3);
	memcpy(imagedata, resizeImage.data, resize_width * resize_height * 3);
	npy_intp Dims[2]= { resize_height, resize_width * 3}; //给定维度信息
	PyObject* PyListRGB = PyArray_SimpleNewFromData(2, Dims, NPY_UBYTE, imagedata);

	PyObject* ArgList = PyTuple_New(3);
	PyTuple_SetItem(ArgList, 0, PyListRGB);//将PyList对象放入PyTuple对象中
	PyTuple_SetItem(ArgList, 1, Py_BuildValue("i", resize_width));
	PyTuple_SetItem(ArgList, 2, Py_BuildValue("i", resize_height));*/

	//struct timeval start_time, end_time;
	//gettimeofday( &start_time, NULL );
	//PyObject* pFunc;
	//pFunc = PyObject_GetAttrString(pModule, "process1");
	PyObject* Pyresult = PyObject_CallObject(pFunc, ArgList);//调用函数，完成传递
	//gettimeofday( &end_time, NULL );
	//printf("detect time =%f\n",(end_time.tv_sec - start_time.tv_sec) + (end_time.tv_usec - start_time.tv_usec)/1000000.0);
    PyObject* ret_objs;
	PyArg_Parse(Pyresult, "O!", &PyList_Type, &ret_objs);
	size = PyList_Size(ret_objs);
	int i, j;
	//得到检测框数据
	for(i = 0; i < size/6; i++)
	{

		int class_id, confidence, x, y, w, h;
		class_id = PyLong_AsLong(PyList_GetItem(ret_objs,i*6+0));
		confidence = PyLong_AsLong(PyList_GetItem(ret_objs,i*6+1));
		x = PyLong_AsLong(PyList_GetItem(ret_objs,i*6+2));
		y = PyLong_AsLong(PyList_GetItem(ret_objs,i*6+3));
		w = PyLong_AsLong(PyList_GetItem(ret_objs,i*6+4));
		h = PyLong_AsLong(PyList_GetItem(ret_objs,i*6+5));
		/*if(class_id == 0)//背景
			continue;//mobile net 0 为背景*/
		rst[i * 6 + 0] = class_id;
		rst[i * 6 + 1] = confidence;
		rst[i * 6 + 2] = x;
		rst[i * 6 + 3] = y;
		rst[i * 6 + 4] = w;
		rst[i * 6 + 5] = h;
		nboxes++;
	}
	printf("nboxes = %d\n",nboxes);
	//PyObject_CallObject(pFunc, NULL);
	free(imagedata);
	if(Pyresult)
	   Py_DECREF(Pyresult);
	if(ArgList)
		Py_DECREF(ArgList);
	//if(pFunc)
	//	Py_DECREF(pFunc);
	Py_UNBLOCK_THREADS;  
	Py_END_ALLOW_THREADS; 
	PyGILState_Release(gstate);    //释放当前线程的GIL
#else
    std::vector<DetectionObject> objs;
    //printf("%s", "[ INFO ] Call intel_dldt_detect\n");
    intel_dldt_detect(img, pCfgs->NCS_ID, objs);
    gettimeofday(&end_time1, NULL );
    nboxes = objs.size();
    for(int i = 0; i < nboxes; i++)
    {
        rst[i * 6 + 0] = objs[i].class_id;
        //std::cout << "confidence " << objs[i].confidence;
        rst[i * 6 + 1] = int(objs[i].confidence*100);
        rst[i * 6 + 2] = objs[i].xmin;
        rst[i * 6 + 3] = objs[i].ymin;
        rst[i * 6 + 4] = objs[i].xmax - objs[i].xmin;
        rst[i * 6 + 5] = objs[i].ymax - objs[i].ymin;
        //printf("confidence=%f;\nxmin=%d;\nymin=%d;\nw=%d;\nh=%d;\n",objs[i].confidence, objs[i].xmin, objs[i].ymin, objs[i].xmax - objs[i].xmin, objs[i].ymax - objs[i].ymin);
    }
    printf("nboxes = %d\n", nboxes);
#endif
	gettimeofday( &end_time2, NULL );
    printf("arith detect time =%f,%f\n",(end_time1.tv_sec - start_time.tv_sec)*1000.0 + (end_time1.tv_usec - start_time.tv_usec)/1000.0, (end_time2.tv_sec - start_time.tv_sec)*1000.0 + (end_time2.tv_usec - start_time.tv_usec)/1000.0);
	return nboxes;

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////计算两个框的交集
int overlapRatio(const CRect r1,const CRect r2)
{
	int ratio=0;
	int x1   =   max(r1.x, r2.x);
	int y1   =   max(r1.y, r2.y);
	int x2   =   min(r1.x + r1.width, r2.x + r2.width);
	int y2   =   min(r1.y + r1.height, r2.y + r2.height);

	if(x1 < x2 && y1 < y2) //intersect
	{
		int area_r1 = r1.width * r1.height;
		int area_r2 = r2.width * r2.height;
		int area_intersection = (x2 - x1) * (y2 - y1);

		ratio = area_intersection * 100 / (area_r1 + area_r2 - area_intersection);
	}
	return ratio;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////找到最近框
CTarget* find_nearest_rect(	CRect detectBox, int class_id, CTarget* targets, int targets_size)
{

	if(targets_size < 1)
	{
		return NULL;
	}
	int i = 0;
	int overlap_ratio = 0;
	if(targets_size)
	{
		CTarget* it_max = &targets[0];
		for(i = 0; i < targets_size; i++)
		{
			//if(class_id == targets[i].class_id)
			{
				int ratio = 0;
				ratio = overlapRatio(detectBox, targets[i].box);
				if(ratio > overlap_ratio)
				{
					overlap_ratio = ratio;
					it_max = &targets[i];
				}
			}
		}
		if(overlap_ratio > 5)
			return it_max;
	}
	return NULL;

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////检测框和目标进行匹配
int match_object_rect(CTarget* targets, int targets_size, CDetBox* detClasses, int class_id, int* match_object, int* match_rect, int thresh)
{

	if(targets_size < 1 || detClasses[class_id].classes_num < 1)
	{
		return -1;
	}
	int i = 0 , j = 0;
	int overlap_ratio = 0, idx_max = 0;
	memset(match_object, -1, MAX_TARGET_NUM * sizeof(int));
	memset(match_rect, -1, MAX_TARGET_NUM * sizeof(int));

	for(j = 0; j < detClasses[class_id].classes_num; j++)//匹配目标
	{
		match_object[j] = -1;
		idx_max = -1;
		overlap_ratio = 0;
		for(i = 0; i < targets_size; i++)
		{
			//if(class_id == targets[i].class_id)
			{
				int ratio = 0;
				ratio = overlapRatio(detClasses[class_id].box[j], targets[i].box);
				if(ratio > overlap_ratio)
				{
					overlap_ratio = ratio;
					idx_max = i;
				}
			}
		}
		if(overlap_ratio > thresh)
			match_object[j] = idx_max;
	}

	for(j = 0; j < targets_size; j++)//匹配框
	{
		match_rect[j] = -1;
		idx_max = -1;
		overlap_ratio = 0;
		for(i = 0; i < detClasses[class_id].classes_num; i++)
		{
			//if(class_id == targets[j].class_id)
			{
				int ratio = 0;
				ratio = overlapRatio(detClasses[class_id].box[i], targets[j].box);
				if(ratio > overlap_ratio)
				{
					overlap_ratio = ratio;
					idx_max = i;
				}
			}
		}
		if(overlap_ratio > thresh)
			match_rect[j] = idx_max;
	}

	return 1;

}
//分析目标轨迹
int analysis_trajectory(CTarget* target)
{
	int direction = 0;
	int dir_x = 0;
	int dir_y = 0;
	if(target->trajectory_num < 2)
	{
		return -1;
	}
	dir_x = target->trajectory[target->trajectory_num - 1].x - target->trajectory[0].x;
	dir_y = target->trajectory[target->trajectory_num - 1].y - target->trajectory[0].y;
	if(dir_y && dir_y > abs(dir_x))
		direction = 0;
	else if(dir_y < 0 && dir_y > abs(dir_x))
		direction = 1;
	else if(dir_x)
		direction = 2;
	else
		direction =3;

	return direction;

}
//得到目标速度
void get_speed(CTarget* target)
{
	int vx = 0, vy = 0;
	int num = 0, idx = 0;
	idx  = target->trajectory_num - 1;
	if(target->trajectory_num > 1)
	{
		while(num < 3)
		{
			vx += target->trajectory[idx].x - target->trajectory[idx - 1].x;
			vy += target->trajectory[idx].y - target->trajectory[idx - 1].y;
			num++;
			idx--;
			if(idx == 0 || num >= 3)
				break;
		}
		target->vx = vx / num;
		target->vy = vy / num;
	}

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////合并检测框
int merge_overlapped_box(CRect detRect, int class_id, float prob, ALGCFGS* pCfgs, Uint16 ratio_threshold)
{
	int i = 0, j = 0, k = 0;
	CRect r1;
	float prob1 = 0.0;
	for(j = 0; j < pCfgs->classes; j++)
	{
		if(pCfgs->detClasses[j].classes_num == 0)
			continue;
		if( j != class_id )
		{
			for( i = 0; i < pCfgs->detClasses[j].classes_num; i++)
			{
				r1 = pCfgs->detClasses[j].box[i];
				prob1 = pCfgs->detClasses[j].prob[i];
				if(overlapRatio(r1, detRect) > ratio_threshold)
				{
					if(prob1 < prob)
					{
						for( k = i + 1; k < pCfgs->detClasses[j].classes_num; k++)
						{
							pCfgs->detClasses[j].box[k - 1] = pCfgs->detClasses[j].box[k];
							pCfgs->detClasses[j].prob[k - 1] = pCfgs->detClasses[j].prob[k];
						}
						pCfgs->detClasses[j].classes_num = pCfgs->detClasses[j].classes_num - 1;
						i--;
						//printf("[%d,%d],[%f,%f],%d\n",pCfgs->detClasses[j].class_id,class_id,prob1,prob,overlapRatio(r1, detRect));
					}
					else
					{
						return 1;
					}
				}
			}
		}
	}
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////对检测框进行后处理
void post_process_box(ALGCFGS* pCfgs,  Uint16 ratio_threshold)
{
	int i = 0, j = 0, k = 0;
	CRect r;
	float prob = 0.0;
	int class_id = 0;
	int val = 0;
	for(j = 0; j < pCfgs->classes; j++)//分类别进行
	{
		if(pCfgs->detClasses[j].classes_num == 0)
			continue;
		for( i = 0; i < pCfgs->detClasses[j].classes_num; i++)
		{
			r = pCfgs->detClasses[j].box[i];
			prob = pCfgs->detClasses[j].prob[i];
			class_id = pCfgs->detClasses[j].class_id;
			val = merge_overlapped_box(r, class_id, prob, pCfgs, ratio_threshold);
			if(val == 1)
			{
				for( k = i + 1; k < pCfgs->detClasses[j].classes_num; k++)
				{
					pCfgs->detClasses[j].box[k - 1] = pCfgs->detClasses[j].box[k];
					pCfgs->detClasses[j].prob[k - 1] = pCfgs->detClasses[j].prob[k];
				}
				//printf("%d,%f\n", pCfgs->detClasses[j].class_id, prob);
				pCfgs->detClasses[j].classes_num = pCfgs->detClasses[j].classes_num - 1;
				i--;

			}
		}
	}
}
inline Uint16 CalTargetSpeed(CTarget target, int laneID, ALGCFGS *pCfgs)//计算目标实际速度
{
	float uVehicleSpeed = 0;
	int pos1 = target.trajectory[0].y;
	int pos2 = target.box.y + target.box.height / 2;
	float len = pCfgs->actual_distance[laneID][pos2] - pCfgs->actual_distance[laneID][pos1];
	len = (len > 0)? len : -1 * len;
	uVehicleSpeed = len * 3.6 / (target.end_time - target.start_time + 1e-6);
	//printf("[%d,%d,%d],len = %f, t = %f,speed = %f,[%f,%f]\n", target.continue_num, pos1, pos2, len, target.end_time - target.start_time, uVehicleSpeed,target.start_time,target.end_time);
/*#ifdef SAVE_VIDEO
	char str[10];
	sprintf(str, "speed:%d", (int)uVehicleSpeed);
	putText(img, str, Point(target.box.x + 50,max(0,target.box.y - 10)), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 255,0 ), 2);
#endif*/
	if(pCfgs->ResultMsg.uResultInfo.uEachLaneData[laneID].SpeedDetectInfo1.uVehicleQueueLength)
		uVehicleSpeed = (uVehicleSpeed > 15)? 15 : uVehicleSpeed;
	else
		uVehicleSpeed = (uVehicleSpeed > 60)? 60 : uVehicleSpeed;
	if(uVehicleSpeed <= 0)
		uVehicleSpeed = rand() % 5 + 1;
	return (Uint16)uVehicleSpeed;
}
inline Uint16 CalTargetLength(CTarget target, int laneID, ALGCFGS *pCfgs)//计算目标实际长度
{
	int pos1 = target.box.y;
	int pos2 = target.box.y + target.box.height ;
	float len = pCfgs->actual_distance[laneID][pos2] - pCfgs->actual_distance[laneID][pos1];
	len = (len > 0)? len : -1 * len;
	if(strcmp(target.names, "car") == 0)
	{
		len = (len < 3)? 3 : len;
		len = (len > 6)? 6 : len;
	}
	else if(strcmp(target.names, "truck") == 0)
	{
		len = (len < 6)? 6 : len;
		len = (len > 13)? 13 : len;
	}
	else
	{
		len = (len < 9)? 9 : len;
		len = (len > 15)? 15 : len;
	}
	return (Uint16)len;
}
//车辆进入线圈，设置参数
void obj_in_region(ALGCFGS *pCfgs, int LaneID, int idx)//0为流量线圈，1为后线圈
{
	pCfgs->headtime[LaneID] = pCfgs->currIime - pCfgs->jgtime[LaneID][idx];
	pCfgs->jgtime[LaneID][idx] = pCfgs->currIime;//用于计算头间距
	pCfgs->headtime[LaneID] = (pCfgs->headtime[LaneID] < 0)? 0 : pCfgs->headtime[LaneID];
	pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.CoilAttribute[idx].uVehicleHeadtime = pCfgs->headtime[LaneID];//车头时距
	pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.CoilAttribute[idx].calarflag = 1;//车在线圈内
	pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.CoilAttribute[idx].DetectInSum++;
}
//车辆离开线圈,连续车辆时，前一车辆不删除目标但是出车，将calarflag设为2
void obj_out_region(ALGCFGS *pCfgs, int LaneID, CTarget* target, int idx)//0为流量线圈，1为后线圈
{
	if(pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.CoilAttribute[idx].calarflag == 1)//车出线圈时，calarflag设置为2
		pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.CoilAttribute[idx].calarflag = 2;
	if(target->cal_speed == FALSE)//未计算速度，计算车型、车速、车长
	{
		pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.CoilAttribute[idx].DetectOutSum++;
		//pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.CoilAttribute[idx].uVehicleType = target.class_id + 1;
		//clock_t end_time = clock(); 
		//target.end_time = (float)end_time / CLOCKS_PER_SEC;
		target->end_time = pCfgs->currIime;
		pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.CoilAttribute[idx].uVehicleSpeed  = CalTargetSpeed(*target, LaneID, pCfgs);
		target->cal_speed = TRUE;
		pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.CoilAttribute[idx].uVehicleLength  = CalTargetLength(*target, LaneID, pCfgs);

	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////初始化目标
bool Initialize_target(CTarget* target)
{
	target->box.x = 0;
	target->box.y = 0;
	target->box.width = 0;
	target->box.height = 0;
	target->prob = 0;
	target->class_id = 0;
	target->detected = FALSE;
	target->target_id = 0;
	target->lost_detected = 0;
	target->trajectory_num = 0;
	target->vx = 0;
	target->vy = 0;
	target->continue_num = 0;
	target->cal_speed = FALSE;
	target->cal_flow = FALSE;
	target->start_time = 0;
	target->end_time = 0;
	target->region_idx = -1;
	return true;
}
inline void DeleteTarget(int* size, int* startIdx, CTarget* target)//删除目标
{
	Uint16 sz = *size;
	Uint16 idx = *startIdx;
	Uint16 j=0;
	for(j = idx + 1; j < sz; j++)
	{
		target[j-1] = target[j];
	}
	*size = sz - 1;
	*startIdx = idx - 1;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////分析行人检测框
void ProcessPersonBox(ALGCFGS* pCfgs, ALGPARAMS *pParams, int width, int height)
{
	int i, j, k;
	int left = 0, right = 0, top = 0, bottom = 0;
	int dis1 = 0, dis2 = 0;
	int x = 0, y = 0;
	int x0 = 0, y0 = 0;
	float val1 = 0, val2 = 0;
	int nPersonNum = 0;
	Uint16 uRegionPersonNum[MAX_REGION_NUM] = {0};
	int match_object[MAX_TARGET_NUM] = { 0 };
	int match_rect[MAX_TARGET_NUM] = { 0 };
	int match_success = -1;
	bool isIntersect = FALSE;
	for(i = 0; i < pCfgs->uDetectRegionNum; i++)
	{
		memset(pCfgs->uPersonDirNum[i], 0, MAX_DIRECTION_NUM * sizeof(Uint16));//初始化区域方向数
	}
	for( i = 0; i < pCfgs->objPerson_size; i++)
	{
		pCfgs->objPerson[i].detected = FALSE;
	}
	//分析行人检测框
	for( i = 0; i < pCfgs->classes; i++)
	{
		strcpy(pCfgs->detClasses[i].names, LABELS[i]);
		if(strcmp(LABELS[i], "person") != 0 )
			continue;
		if(pCfgs->detClasses[i].classes_num)
		{
			match_object_rect(pCfgs->objPerson, pCfgs->objPerson_size, pCfgs->detClasses, i, match_object, match_rect, 5);
			for( j = 0; j < pCfgs->detClasses[i].classes_num; j++)
			{
				bool inRegion = FALSE;
				//判断行人是否在行人检测区域内
				for(k = 0; k < pCfgs->uDetectRegionNum; k++)
				{
					CRect bx;
					bx = pCfgs->detClasses[i].box[j];
					bx.x = bx.x * pCfgs->m_iWidth /width;
					bx.y = bx.y * pCfgs->m_iHeight /height;
					bx.width = bx.width * pCfgs->m_iWidth /width;
					bx.height = bx.height * pCfgs->m_iHeight /height;
					int isInRegion = RectInRegion(pParams->MaskDetectImage, pCfgs->m_iWidth, pCfgs->m_iHeight, bx, k);
					if(isInRegion > 10)//在检测区域内
					{
						inRegion = TRUE;//在区域内
						//pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum] = pCfgs->detClasses[i].box[j];
						/*pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].x = pCfgs->detClasses[i].box[j].x;
						pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].y = pCfgs->detClasses[i].box[j].y;
						pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].w = pCfgs->detClasses[i].box[j].width;
						pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].h = pCfgs->detClasses[i].box[j].height;
						pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].label = 0;
						pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].confidence = pCfgs->detClasses[i].prob[j];
						pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].id = 0;
						int pos_x = max(0, pCfgs->detClasses[i].box[j].x + pCfgs->detClasses[i].box[j].width / 2);
						pos_x = min(pos_x, width - 1);
						int pos_y =max(0, pCfgs->detClasses[i].box[j].y + pCfgs->detClasses[i].box[j].height / 2);
						pos_y = min(pos_y, height - 1);
						pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].distance = pCfgs->image_actual[pos_y][pos_x][1];//目标与相机的垂直距离
						pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].laneid = -1;//车道ID*/

#ifdef SAVE_VIDEO
						cv::rectangle(img, cv::Rect(pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].x,pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].y,pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].width,pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].height), cv::Scalar(0, 0 ,255), 1, 8, 0 );
#endif
						uRegionPersonNum[k]++;//相应区域行人数加1
						//nPersonNum++;
					}
				}
				if(pCfgs->bDetPersonFlow == FALSE)//不检测行人方向流量
					continue;
				//判断行人是否与检测线相交
				left = pCfgs->detClasses[i].box[j].x;
				right = pCfgs->detClasses[i].box[j].x + pCfgs->detClasses[i].box[j].width;
				top = pCfgs->detClasses[i].box[j].y;
				bottom = pCfgs->detClasses[i].box[j].y + pCfgs->detClasses[i].box[j].height;
				/*CTarget* t;
				t = find_nearest_rect(pCfgs->detClasses[i].box[j], pCfgs->detClasses[i].class_id, pCfgs->objPerson, pCfgs->objPerson_size);
				if(t)//与已有目标匹配成功
				{
					t->box = pCfgs->detClasses[i].box[j];
					t->prob = pCfgs->detClasses[i].prob[j];
					t->class_id = pCfgs->detClasses[i].class_id;
					t->detected = TRUE;
				}
				else */
				match_success = -1;
				for( k = 0; k < pCfgs->objPerson_size; k++)
				{
					if(match_object[j] == k && match_rect[k] == j)//与目标匹配
					{
						match_success = 1;
						pCfgs->objPerson[k].box = pCfgs->detClasses[i].box[j];
						pCfgs->objPerson[k].prob = pCfgs->detClasses[i].prob[j];
						pCfgs->objPerson[k].class_id = pCfgs->detClasses[i].class_id;
						pCfgs->objPerson[k].detected = TRUE;
						break;
					}
				}
				if(match_success < 0)//未匹配
				{
					//与检测线相交加入目标链
					isIntersect = FALSE;
					for(k = 0; k < pCfgs->uDetectRegionNum; k++)//逐个与检测线判断，找到与检测线相交，并加入到目标
					{
						dis1 = min(pCfgs->detLineParm[k].detRight, right) - max(pCfgs->detLineParm[k].detLeft, left);
						//dis2 = min(pCfgs->detLineParm[k].detBottom, bottom) - max(pCfgs->detLineParm[k].detTop, top);
						val1 = pCfgs->detLineParm[k].k * left + pCfgs->detLineParm[k].b - top;
						val2 = pCfgs->detLineParm[k].k * right + pCfgs->detLineParm[k].b - bottom;
						//if(dis1 >= 0 && val1 * val2 <= 0 && pCfgs->objPerson_size < MAX_TARGET_NUM)//与检测线相交
						if(inRegion && pCfgs->objPerson_size < MAX_TARGET_NUM)//在检测区域内，加入到新目标
						{
							CTarget nt; 
							Initialize_target(&nt);
							nt.box = pCfgs->detClasses[i].box[j];
							nt.class_id = pCfgs->detClasses[i].class_id;
							nt.prob = pCfgs->detClasses[i].prob[j];
							nt.detected = TRUE;
							nt.target_id = pCfgs->person_id++;
							nt.region_idx = k;//行人与哪个检测线相交
							if(pCfgs->person_id > 5000)
								pCfgs->person_id = 1;
							strcpy(nt.names, pCfgs->detClasses[i].names);
							pCfgs->objPerson[pCfgs->objPerson_size] = nt;
							pCfgs->objPerson_size++;
							break;
						}
					}
				}
			}
		}
	}
	//分析行人目标
	for(i = 0; i < pCfgs->objPerson_size; i++)
	{
#ifdef SAVE_VIDEO
		int flag = 0;
#endif
		int region_idx = pCfgs->objPerson[i].region_idx;
		//保存轨迹，轨迹数小于3000，直接保存，大于3000，去除旧的
		if(pCfgs->objPerson[i].trajectory_num < 3000)
		{

			pCfgs->objPerson[i].trajectory[pCfgs->objPerson[i].trajectory_num].x = pCfgs->objPerson[i].box.x + pCfgs->objPerson[i].box.width / 2;
			pCfgs->objPerson[i].trajectory[pCfgs->objPerson[i].trajectory_num].y = pCfgs->objPerson[i].box.y + pCfgs->objPerson[i].box.height / 2;
			pCfgs->objPerson[i].trajectory_num++;
		}
		else
		{
			for(j = 0; j < pCfgs->objPerson[i].trajectory_num - 1; j++)
			{
				pCfgs->objPerson[i].trajectory[j] = pCfgs->objPerson[i].trajectory[j + 1];
			}
			pCfgs->objPerson[i].trajectory[pCfgs->objPerson[i].trajectory_num - 1].x = pCfgs->objPerson[i].box.x + pCfgs->objPerson[i].box.width / 2;
			pCfgs->objPerson[i].trajectory[pCfgs->objPerson[i].trajectory_num - 1].y = pCfgs->objPerson[i].box.y + pCfgs->objPerson[i].box.height / 2;
		}

		//检测到，并更新速度
		if(pCfgs->objPerson[i].detected)
		{
			pCfgs->objPerson[i].lost_detected = 0;
            //get_speed(&pCfgs->objPerson[i]);
		}
		else//未检测到
		{
			pCfgs->objPerson[i].lost_detected++;
			pCfgs->objPerson[i].box.x += pCfgs->objPerson[i].vx;
			pCfgs->objPerson[i].box.y += pCfgs->objPerson[i].vy;
		}

		//判断是否计数
		x = pCfgs->objPerson[i].box.x + pCfgs->objPerson[i].box.width / 2 ;
		y = pCfgs->objPerson[i].box.y + pCfgs->objPerson[i].box.height / 2 ;
		x0 = pCfgs->objPerson[i].trajectory[0].x;
		y0 = pCfgs->objPerson[i].trajectory[0].y;
		if(pCfgs->objPerson[i].cal_flow == FALSE)
		{
			val1 = pCfgs->detLineParm[region_idx].k * x + pCfgs->detLineParm[region_idx].b - y;
			val2 = pCfgs->detLineParm[region_idx].k * x0 + pCfgs->detLineParm[region_idx].b - y0;
			if(val1 * val2 < 0)//从检测线一侧运动到另一侧
			{
				printf("get person obj\n");
				get_object_num(pCfgs, i, region_idx);
				pCfgs->objPerson[i].cal_flow = TRUE;
				//printf("%d,count 1\n",pCfgs->gThisFrameTime);
#ifdef SAVE_VIDEO
				flag = 1;
#endif
			}
		}
		left = pCfgs->objPerson[i].box.x;
		right = pCfgs->objPerson[i].box.x + pCfgs->objPerson[i].box.width;
		top = pCfgs->objPerson[i].box.y;
		bottom = pCfgs->objPerson[i].box.y + pCfgs->objPerson[i].box.height;
		dis1 = min(pCfgs->detLineParm[region_idx].detRight + 5, right) - max(pCfgs->detLineParm[region_idx].detLeft - 5, left);
		dis2 = min(pCfgs->detLineParm[region_idx].detBottom + 5, bottom) - max(pCfgs->detLineParm[region_idx].detTop - 5, top);
		//val1 = pCfgs->detLineParm.k * left + pCfgs->detLineParm.b - top;
		//val2 = pCfgs->detLineParm.k * right + pCfgs->detLineParm.b - bottom;
		//去除不在检测区域的目标
		if(dis1 >= 0 && dis2 >= 0)
		//if(dis1 >= 0 && val1 * val2 <= 0)
		{
			;
		}
		else if(pCfgs->objPerson[i].continue_num > 5)//目标存在大于5帧，防止检测框位置跳动
		{
	       /* if(pCfgs->objPerson[i].cal_flow == FALSE)//这里统计容易多检
			{
				get_object_num(pCfgs, i);
				pCfgs->objPerson[i].cal_flow = TRUE;
#ifdef SAVE_VIDEO
				flag = 2;
#endif
				//printf("%d,count 2\n",pCfgs->gThisFrameTime);
			}*/
			//DeleteTarget(&pCfgs->objPerson_size, &i, pCfgs->objPerson);
			//continue;
		}
		//当目标在视频存在时间太长或长时间没有检测到或离开图像，删除目标
		if(pCfgs->objPerson[i].continue_num > 5000 || pCfgs->objPerson[i].lost_detected > 5 || ((left < 10 || top < 10 || right > (width - 10)  || bottom > (height - 10))&& pCfgs->objPerson[i].lost_detected > 2))
		{
			/*if(pCfgs->objPerson[i].cal_flow == FALSE)
			{
				get_object_num(pCfgs, i);
			    pCfgs->objPerson[i].calflow = TRUE;
			}*/
			
            DeleteTarget(&pCfgs->objPerson_size, &i, pCfgs->objPerson);
			continue;

		}
		//保存行人目标框
		if(pCfgs->objPerson[i].detected)
		{
			pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].x = pCfgs->objPerson[i].box.x;
			pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].y = pCfgs->objPerson[i].box.y;
			pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].w = pCfgs->objPerson[i].box.width;
			pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].h = pCfgs->objPerson[i].box.height;
			pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].label = 0;
			pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].confidence = pCfgs->objPerson[i].prob;
			pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].id =pCfgs->objPerson[i].target_id;//目标id
			int pos_x = max(0, pCfgs->objPerson[i].box.x + pCfgs->objPerson[i].box.width / 2);
			pos_x = min(pos_x, width - 1);
			int pos_y =max(0, pCfgs->objPerson[i].box.y + pCfgs->objPerson[i].box.height / 2);
			pos_y = min(pos_y, height - 1);
			pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].distance = pCfgs->image_actual[pos_y][pos_x][1];//目标与相机的垂直距离
			pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].laneid = -1;//车道ID
			/*if(pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].w <= 0 || pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].h <= 0 || pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].w > width || pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].h > height)
			{
				prt(info," person box width = %d, height = %d",pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].w, pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].h);
			}*/
			prt(info," person box width = %d, height = %d",pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].w, pCfgs->ResultMsg.uResultInfo.udetPersonBox[nPersonNum].h);
			nPersonNum++;
		}

		pCfgs->objPerson[i].continue_num++;
#ifdef SAVE_VIDEO
		if(flag == 0)
			cv::rectangle(img, cv::Rect(pCfgs->objPerson[i].box.x - 2,pCfgs->objPerson[i].box.y - 2 ,pCfgs->objPerson[i].box.width + 4,pCfgs->objPerson[i].box.height + 4), cv::Scalar(255, 255 ,255), 1, 8, 0 );
		else if(flag == 1)
			cv::rectangle(img, cv::Rect(pCfgs->objPerson[i].box.x - 2,pCfgs->objPerson[i].box.y - 2 ,pCfgs->objPerson[i].box.width + 4,pCfgs->objPerson[i].box.height + 4), cv::Scalar(255, 0 ,0), 1, 8, 0 );
		else
			cv::rectangle(img, cv::Rect(pCfgs->objPerson[i].box.x - 2,pCfgs->objPerson[i].box.y - 2 ,pCfgs->objPerson[i].box.width + 4,pCfgs->objPerson[i].box.height + 4), cv::Scalar(0, 255 ,0), 1, 8, 0 );
		char str[10];
		sprintf(str, "%d", pCfgs->objPerson[i].target_id);
		putText(img, str, cv::Point(pCfgs->objPerson[i].box.x,max(0,pCfgs->objPerson[i].box.y - 10)), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 2);
#endif

	}
	pCfgs->ResultMsg.uResultInfo.udetPersonNum = nPersonNum;
	printf("person num = %d\n",nPersonNum);
	//统计行人
	nPersonNum = 0;
	for(i = 1; i >= 0; i--)
	{
		pCfgs->uStatPersonNum[i + 1] = pCfgs->uStatPersonNum[i];
	}
	pCfgs->uStatPersonNum[0] = pCfgs->ResultMsg.uResultInfo.udetPersonNum;
	if(pCfgs->uStatPersonNum[3] < 3)
	{
		pCfgs->uStatPersonNum[3]++;
	}
	for(i = 0; i < pCfgs->uStatPersonNum[3]; i++)
	{
		nPersonNum = nPersonNum + pCfgs->uStatPersonNum[i];
	}
	pCfgs->ResultMsg.uResultInfo.udetStatPersonNum = (float)nPersonNum / pCfgs->uStatPersonNum[3];
#ifdef SAVE_VIDEO
	char str1[10];
	sprintf(str1, "%d", pCfgs->uPersonDirNum[0][0]);
	putText(img, str1, cv::Point(100,30), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 2);
	char str2[10];
	sprintf(str2, "%d", pCfgs->uPersonDirNum[0][1]);
	putText(img, str2, cv::Point(150,30), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 2);
	cv::line(img, cv::Point(pCfgs->detLineParm.pt[0].x,pCfgs->detLineParm.pt[0].y),cv::Point(pCfgs->detLineParm.pt[1].x,pCfgs->detLineParm.pt[1].y), cv::Scalar(255, 0 ,0), 3, 8, 0 );
#endif
	printf("person obj =%d,direction num = [%d,%d]\n",pCfgs->objPerson_size,pCfgs->uPersonDirNum[0][0], pCfgs->uPersonDirNum[0][1]);
	memcpy(pCfgs->uRegionPersonNum, uRegionPersonNum, MAX_REGION_NUM * sizeof(Uint16));//区域行人数
}
//处理检测框
void  get_target(Mat img, int* result, int nboxes, ALGCFGS *pCfgs, ALGPARAMS *pParams, int laneNum, int imgW, int imgH)
{
	//printf("get_target start...................................\n");
	CPoint ptCorner[MAX_LANE][16];
	int i = 0, j = 0, k = 0;
	int left = 0,right = 0, top = 0, bottom = 0;
	int x1 = 0, x2 = 0, x3 = 0;
	int maxValue = 0;
	int maxLane = 0;
	int dis = 0;
	int classes_num[MAX_CLASSES] = { 0 };
	int class_id = 0;
	int lane_id = 0;
	int nboxes1 = 0;
	int vehicle_num[MAX_LANE] = { 0 };
	int vehicle_num1[MAX_LANE] = {0};
	bool obj_lost[MAX_LANE] = { FALSE };
	int match_object[MAX_TARGET_NUM] = { 0 };
	int match_rect[MAX_TARGET_NUM] = { 0 };
	int match_success = -1, match_obj_idx = 0;
	int overlap_x = 0, overlap_y = 0;
	int det_match_object[MAX_TARGET_NUM] = { 0 };
	int det_match_rect[MAX_TARGET_NUM] = { 0 };
	int det_match_success = -1, det_match_obj_idx = 0;
	float sum = 0.0;
	int dis_x = 0, dis_y = 0;
	int laneStatus[MAX_LANE][2] = { 0 };//车道占有状态
	//车道坐标信息
	for( i = 0; i < laneNum; i++)
	{
		ptCorner[i][0] = pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[0];
		ptCorner[i][1] = pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[1];
		ptCorner[i][2] = pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[2];
		ptCorner[i][3] = pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[3];
		ptCorner[i][4] = pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[0];
		ptCorner[i][5] = pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[1];
		ptCorner[i][6] = pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[2];
		ptCorner[i][7] = pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[3];
		ptCorner[i][8] = pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[0];
		ptCorner[i][9] = pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[1];
		ptCorner[i][10] = pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[2];
		ptCorner[i][11] = pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[3];
		ptCorner[i][12] = pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[0];
		ptCorner[i][13] = pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[1];
		ptCorner[i][14] = pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[2];
		ptCorner[i][15] = pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[3];
		laneStatus[i][0] = 0;
		laneStatus[i][1] = 0;
		if(pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[0].calarflag)
			pCfgs->existFrameNum[i][0]++;
		else
			pCfgs->existFrameNum[i][0] = 0;
		if(pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[1].calarflag)
			pCfgs->existFrameNum[i][1]++;
		else
			pCfgs->existFrameNum[i][1] = 0;
		if(pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[0].calarflag == 2)//流量线圈上一帧为出车状态，先置为0
		{
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[0].calarflag = 0;
		}
		if(pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[1].calarflag == 2)//后线圈上一帧为出车状态，先置为0
		{
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[1].calarflag = 0;
		}
	}
	memset(pCfgs->Tailposition, 0, laneNum * sizeof(Uint16));//头车位置
	memset(pCfgs->Headposition, 10000, laneNum * sizeof(Uint16));//末车位置
	//memset(pCfgs->uDetectVehicleSum, 0, laneNum * sizeof(Uint32));
	memset(pCfgs->IsCarInTail, 0, laneNum * sizeof(bool));//尾部占有状态

	memset(pCfgs->detClasses, 0, MAX_CLASSES * sizeof(CDetBox));//初始化检测框
	memset(classes_num, 0, MAX_CLASSES * sizeof(int));//初始化检测框数
	for( i = 0; i < pCfgs->targets_size; i++)//设置未检测
	{
		pCfgs->targets[i].detected = FALSE;
	}
	for( i = 0; i < pCfgs->detTargets_size; i++)//设置未检测
	{
		pCfgs->detTargets[i].detected = FALSE;
	}

	//把检测结果按照类别归类
	for(i = 0; i < nboxes; i++)
	{
		class_id = result[i * 6];
		//if( strcmp(pCfgs->names[class_id],"person") != 0 && strcmp(pCfgs->names[class_id],"bicycle") != 0 && strcmp(pCfgs->names[class_id],"car") != 0 && strcmp(pCfgs->names[class_id],"motorbike") != 0 && strcmp(pCfgs->names[class_id],"bus") != 0 && strcmp(pCfgs->names[class_id],"train") != 0 && strcmp(pCfgs->names[class_id],"truck") != 0)
		//	continue;
		//去掉不符合条件的检测框
		if(result[i * 6 + 4] > 400 || result[i * 6 + 5] > 400 || result[i * 6 + 4] <= 0 || result[i * 6 + 5] <= 0)
			continue;
		pCfgs->detClasses[class_id].class_id = class_id;
		pCfgs->detClasses[class_id].prob[classes_num[class_id]] = result[i * 6 + 1];
		pCfgs->detClasses[class_id].box[classes_num[class_id]].x = result[i * 6 + 2];
		pCfgs->detClasses[class_id].box[classes_num[class_id]].y = result[i * 6 + 3];
		pCfgs->detClasses[class_id].box[classes_num[class_id]].width = result[i * 6 + 4];
		pCfgs->detClasses[class_id].box[classes_num[class_id]].height = result[i * 6 + 5];
		pCfgs->detClasses[class_id].lane_id[classes_num[class_id]] = -1;

/*#ifdef SAVE_VIDEO
		cv::rectangle(img, cv::Rect(pCfgs->ResultMsg.uResultInfo.udetBox[i].x,pCfgs->ResultMsg.uResultInfo.udetBox[i].y,pCfgs->ResultMsg.uResultInfo.udetBox[i].width,pCfgs->ResultMsg.uResultInfo.udetBox[i].height), cv::Scalar(0, 0 ,255), 1, 8, 0 );
#endif*/
		classes_num[class_id]++;
		pCfgs->detClasses[class_id].classes_num = classes_num[class_id];
	}
	//对检测框进行后处理
	post_process_box(pCfgs, 60);

	//分析行人检测框
	ProcessPersonBox(pCfgs, pParams, imgW, imgH);
	//分析车辆检测框
	for( i = 0; i < pCfgs->classes; i++)
	{
		strcpy(pCfgs->detClasses[i].names, LABELS[i]);
		if(strcmp(LABELS[i], "car") != 0 && strcmp(LABELS[i], "bus") != 0 && strcmp(LABELS[i], "truck") != 0)
			continue;
		if(pCfgs->detClasses[i].classes_num)
		{
			//目标和检测框进行匹配
			match_object_rect(pCfgs->targets, pCfgs->targets_size, pCfgs->detClasses, i, match_object, match_rect, 15);
			match_object_rect(pCfgs->detTargets, pCfgs->detTargets_size, pCfgs->detClasses, i, det_match_object, det_match_rect, 5);
			for( j = 0; j < pCfgs->detClasses[i].classes_num; j++)
			{
				//目标和检测框匹配，更新目标信息
				det_match_success = -1;
				for( k = 0; k < pCfgs->detTargets_size; k++)
				{
					if(det_match_object[j] == k && det_match_rect[k] == j)
					{
						det_match_success = 1;
						break;
					}
				}
				if(det_match_success == 1)
				{
					pCfgs->detTargets[k].box = pCfgs->detClasses[i].box[j];
					pCfgs->detTargets[k].prob = pCfgs->detClasses[i].prob[j];
					pCfgs->detTargets[k].class_id = pCfgs->detClasses[i].class_id;
					strcpy(pCfgs->detTargets[k].names, pCfgs->detClasses[i].names);
					pCfgs->detTargets[k].detected = TRUE;
					det_match_obj_idx = k;
				}
				match_success = -1;
				for( k = 0; k < pCfgs->targets_size; k++)
				{
					if(match_object[j] == k && match_rect[k] == j)
					{
						match_success = 1;
						pCfgs->targets[k].box = pCfgs->detClasses[i].box[j];
						pCfgs->targets[k].prob = pCfgs->detClasses[i].prob[j];
						pCfgs->targets[k].class_id = pCfgs->detClasses[i].class_id;
						strcpy(pCfgs->targets[k].names, pCfgs->detClasses[i].names);
						pCfgs->targets[k].detected = TRUE;
						break;
					}
				}
				int overlapNum[MAX_LANE] = {-1};
				left = max(0, pCfgs->detClasses[i].box[j].x);
				right = min(pCfgs->detClasses[i].box[j].x + pCfgs->detClasses[i].box[j].width, imgW - 1);
				top = max(0, pCfgs->detClasses[i].box[j].y);
				bottom = min(pCfgs->detClasses[i].box[j].y + pCfgs->detClasses[i].box[j].height, imgH - 1);
				for( k = 0; k < laneNum; k++)//计算与车道相交值
				{
					/*x1 = (float)((top + bottom) / 2 - ptCorner[k][0].y) * (float)(ptCorner[k][3].x - ptCorner[k][0].x) / (float)(ptCorner[k][3].y - ptCorner[k][0].y) + ptCorner[k][0].x;
					x2 = (float)((top + bottom) / 2 - ptCorner[k][1].y) * (float)(ptCorner[k][2].x - ptCorner[k][1].x) / (float)(ptCorner[k][2].y - ptCorner[k][1].y) + ptCorner[k][1].x;
					x3 = min(x2, right)-max(x1, left);
					overlapNum[k] = x3;*/
					CRect bx;
					bx = pCfgs->detClasses[i].box[j];
					bx.x = bx.x * pCfgs->m_iWidth /imgW;
					bx.y = bx.y * pCfgs->m_iHeight /imgH;
					bx.width = bx.width * pCfgs->m_iWidth /imgW;
					bx.height = bx.height * pCfgs->m_iHeight /imgH;
					overlapNum[k] = RectInRegion(pParams->MaskLaneImage, pCfgs->m_iWidth, pCfgs->m_iHeight, bx, k);
				}
				//找出相交最大车道
				maxValue = overlapNum[0];
				maxLane = 0;
				for( k = 1; k < laneNum; k++)
				{
					if(maxValue < overlapNum[k])
					{
						maxValue = overlapNum[k];
						maxLane = k;
					}
				}
				//if(/*maxValue > 0*/maxValue >= (right - left) / 4)//(right - left) / 4
				if(maxValue > 10)//检测框10%之上在车道内
				{
					dis_x = min(max(ptCorner[maxLane][1].x, ptCorner[maxLane][2].x), right) - max(min(ptCorner[maxLane][0].x, ptCorner[maxLane][3].x), left);
					dis_y = min(max(ptCorner[maxLane][2].y, ptCorner[maxLane][3].y), bottom) - max(min(ptCorner[maxLane][0].y, ptCorner[maxLane][1].y), top);
					if(det_match_success == 1)
					{
						pCfgs->detTargets[det_match_obj_idx].lane_id = maxLane;
					}
					if( dis_x > 10 && dis_y > 10)//在占有区域到车道区域下端进行计数
					{
						/*pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].x = pCfgs->detClasses[i].box[j].x;
						pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].y = pCfgs->detClasses[i].box[j].y;
						pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].w = pCfgs->detClasses[i].box[j].width;
						pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].h = pCfgs->detClasses[i].box[j].height;
						pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].label = pCfgs->detClasses[i].class_id + 1;
						pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].confidence = pCfgs->detClasses[i].prob[j];
#ifdef SAVE_VIDEO
		cv::rectangle(img, cv::Rect(pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].x,pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].y,pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].w,pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].h), cv::Scalar(0, 0 ,255), 1, 8, 0 );
#endif
						nboxes1++;*/
						//vehicle_num[maxLane]++;
						//pCfgs->uDetectVehicleSum[maxLane]++;//区域车辆数
						//未匹配，增加新的目标
						if(det_match_success < 0)
						{
							CTarget nt; 
							Initialize_target(&nt);
							nt.box = pCfgs->detClasses[i].box[j];
							nt.class_id = pCfgs->detClasses[i].class_id;
							nt.prob = pCfgs->detClasses[i].prob[j];
							nt.detected = TRUE;
							nt.target_id = pCfgs->detTarget_id++;
							nt.lane_id = maxLane;
							if(pCfgs->detTarget_id > 5000)
								pCfgs->detTarget_id = 1;
							nt.start_time = pCfgs->currIime;
							strcpy(nt.names, pCfgs->detClasses[i].names);
							pCfgs->detTargets[pCfgs->detTargets_size] = nt;
							pCfgs->detTargets_size++;
						}
					}
					pCfgs->detClasses[i].lane_id[j] = maxLane;
					vehicle_num1[maxLane]++;
					pCfgs->Headposition[maxLane] = (pCfgs->Headposition[maxLane] > top)? top : pCfgs->Headposition[maxLane];//头车位置
					pCfgs->Tailposition[maxLane] = (pCfgs->Tailposition[maxLane] < bottom)? bottom : pCfgs->Tailposition[maxLane];//末车位置
					if(pCfgs->IsCarInTail[maxLane] == FALSE)
					{
						if(min(bottom,  ptCorner[maxLane][6].y) - max(top, ptCorner[maxLane][4].y) > 5 && min(right,  ptCorner[maxLane][5].x) - max(left, ptCorner[maxLane][4].x) > 5)//尾部占有状态
						{
							pCfgs->IsCarInTail[maxLane] = TRUE;
							//pCfgs->ResultMsg.uResultInfo.uEachLaneData[maxLane].SpeedDetectInfo1.CoilAttribute[1].calarflag = 1;

						}
					}
					//与流量区域相交
					dis_x = min(ptCorner[maxLane][13].x, right) - max(ptCorner[maxLane][12].x, left);
					dis_y = min(ptCorner[maxLane][14].y, bottom) - max(ptCorner[maxLane][12].y, top);
					//if(min(ptCorner[maxLane][14].y, bottom) - max(ptCorner[maxLane][12].y, top) > 0)
					{
						if(match_success > 0)//匹配成功
						{
							;
						}
						else if(dis_x > 10 && dis_y > 10)//检测框与流量相交，增加新目标
						{
							if(top > max(ptCorner[maxLane][12].y, ptCorner[maxLane][2].y - 100))//防止误检,车top已经进入线圈，不再加入到目标中
								continue;
							/*for( k = 0; k < pCfgs->targets_size; k++)
							{
								if(pCfgs->targets[k].target_id == pCfgs->currFore_target_id[maxLane])
								{
									if(pCfgs->targets[k].cal_flow == FALSE)
									{
										break;
									}
								}
							}
							if(k < pCfgs->targets_size)//当前车道的目标ID还没有进行流量统计
								continue;*/
							if(pCfgs->ResultMsg.uResultInfo.uEachLaneData[maxLane].SpeedDetectInfo1.CoilAttribute[0].calarflag == 1)//此车道存在车
							{
								dis = min(ptCorner[maxLane][14].y, bottom) - max(ptCorner[maxLane][12].y, top);
								if(dis > (ptCorner[maxLane][14].y - ptCorner[maxLane][12].y) / 2)//当检测框在流量线圈占线圈一半以上，不删除目标，但加入新目标
								//if(bottom > (ptCorner[maxLane][12].y + ptCorner[maxLane][14].y) / 2)//不删除目标，但加入新目标
								{
									for( k = 0; k < pCfgs->targets_size; k++)
									{
										if(pCfgs->targets[k].target_id == pCfgs->currFore_target_id[maxLane])
										{
											break;
										}
									}
									if(k < pCfgs->targets_size)//先出车，再进车
									{
										obj_out_region(pCfgs, maxLane, &pCfgs->targets[k], 0);//流量线圈出车
									}
									if( k == pCfgs->targets_size)
									{
										pCfgs->ResultMsg.uResultInfo.uEachLaneData[maxLane].SpeedDetectInfo1.CoilAttribute[0].calarflag = 2;
									}
								}
								else//防止误检，不增加新目标
									continue;
							}
							//增加新的目标
							CTarget nt; 
							Initialize_target(&nt);
							nt.box = pCfgs->detClasses[i].box[j];
							nt.class_id = pCfgs->detClasses[i].class_id;
							nt.prob = pCfgs->detClasses[i].prob[j];
							nt.detected = TRUE;
							nt.target_id = pCfgs->target_id++;
							nt.lane_id = maxLane;
							//clock_t start_time = clock();
							//nt.start_time = (float)start_time / CLOCKS_PER_SEC;
							nt.start_time = pCfgs->currIime;
							if(pCfgs->target_id > 5000)
								pCfgs->target_id = 1;
							strcpy(nt.names, pCfgs->detClasses[i].names);
							//printf("%s\n",pCfgs->detClasses[i].names);
							pCfgs->targets[pCfgs->targets_size] = nt;
							pCfgs->targets_size++;
							//obj_in_region(pCfgs, maxLane, 0);//车入
							pCfgs->currFore_target_id[maxLane] = nt.target_id;

						}
					}
				}
				else
				{
					pCfgs->detClasses[i].lane_id[j] = -1;
				}
			}
		}
	}
	//pCfgs->ResultMsg.uResultInfo.udetNum = nboxes1;    
	//分析目标
	for(i = 0;i < pCfgs->targets_size; i++)
	{
		//保存轨迹
		pCfgs->targets[i].trajectory[pCfgs->targets[i].trajectory_num].x = pCfgs->targets[i].box.x + pCfgs->targets[i].box.width / 2;
		pCfgs->targets[i].trajectory[pCfgs->targets[i].trajectory_num].y = pCfgs->targets[i].box.y + pCfgs->targets[i].box.height / 2;
		pCfgs->targets[i].trajectory_num++;
		//检测到，并更新速度
		if(pCfgs->targets[i].detected)
		{
			pCfgs->targets[i].lost_detected = 0;
            get_speed(&pCfgs->targets[i]);
		}
		else//未检测到
		{
			pCfgs->targets[i].lost_detected++;
			pCfgs->targets[i].box.x += pCfgs->targets[i].vx;
			pCfgs->targets[i].box.y += pCfgs->targets[i].vy;
		}

		lane_id = pCfgs->targets[i].lane_id;
		left = pCfgs->targets[i].box.x;
		right = pCfgs->targets[i].box.x + pCfgs->targets[i].box.width;
		top = pCfgs->targets[i].box.y;
		bottom = pCfgs->targets[i].box.y + pCfgs->targets[i].box.height;
		//目标进入流量线圈进行计数
		if(pCfgs->currFore_target_id[lane_id] == pCfgs->targets[i].target_id && pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[0].calarflag == 0 && pCfgs->targets[i].cal_flow == FALSE)//车刚入线圈
		{
			obj_in_region(pCfgs, lane_id, 0);//流量线圈车入
			pCfgs->targets[i].cal_flow = TRUE;
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[0].uVehicleType = pCfgs->targets[i].class_id + 1;
/*#ifdef DETECT_PLATE//车牌识别
			PlateInfo plateInfo[MAX_PLATE_NUM];
			if(pCfgs->plate_flag >= 0)
			{
				//将目标区域限制在图像范围内
				Rect rct;
				rct.x = MAX(0, pCfgs->targets[i].box.x);
				rct.y = MAX(0, pCfgs->targets[i].box.y);
				rct.width = MIN(pCfgs->targets[i].box.x + pCfgs->targets[i].box.width, imgW - 1) - rct.x;
				rct.height = MIN(pCfgs->targets[i].box.y + pCfgs->targets[i].box.height, imgH - 1) - rct.y;
				Mat roiImg = img(rct);//获得目标区域图像
				int plateNum = PlateDetectandRecognize(roiImg.data, rct.width, rct.height, 0, plateInfo, pCfgs->plate_flag);//车牌识别
				for(j = 0; j < plateNum; j++)
				{
					plateInfo[j].plateRect.x += rct.x;
					plateInfo[j].plateRect.y += rct.y;
				}
				printf("plate num = %d\n",plateNum);
			}
#endif*/
		}
		//计算车速、车长
		if(pCfgs->currFore_target_id[lane_id] == pCfgs->targets[i].target_id && pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[0].calarflag == 1)//车已经进入线圈
		{
			dis = min(ptCorner[lane_id][14].y, bottom) - max(ptCorner[lane_id][12].y, top);
			if(dis >= min(pCfgs->targets[i].box.height, ptCorner[lane_id][14].y - ptCorner[lane_id][12].y - 2) && pCfgs->targets[i].cal_speed == FALSE && pCfgs->targets[i].continue_num)//当车辆到达流量区域下边缘
				//if(pCfgs->targets[i].box.y + pCfgs->targets[i].box.height > (ptCorner[lane_id][14].y + ptCorner[lane_id][15].y)/2 && pCfgs->targets[i].cal_speed == FALSE && pCfgs->targets[i].continue_num)//当车辆到达流量区域下边缘
			{
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[0].DetectOutSum++;
				//pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[0].uVehicleType = pCfgs->targets[i].class_id + 1;
				//clock_t end_time = clock();
				//pCfgs->targets[i].end_time = (float)end_time / CLOCKS_PER_SEC;
				pCfgs->targets[i].end_time = pCfgs->currIime;
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[0].uVehicleSpeed  = CalTargetSpeed(pCfgs->targets[i], lane_id, pCfgs);
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[0].uVehicleLength  = CalTargetLength(pCfgs->targets[i], lane_id, pCfgs);
				pCfgs->targets[i].cal_speed = TRUE;
			}
		}
		//防止框跳动，出车
		/*top = min(ptCorner[lane_id][12].y - 10, ptCorner[lane_id][13].y - 10);
		top = max(0, top);
		bottom = max(ptCorner[lane_id][14].y + 10, ptCorner[lane_id][15].y + 10);
		bottom = min(imgH - 1, bottom);
		dis = min(bottom, pCfgs->targets[i].box.y + pCfgs->targets[i].box.height) - max(top, pCfgs->targets[i].box.y);*/
		//计算与流量区域是否相交
		dis = min(ptCorner[lane_id][14].y, bottom) - max(ptCorner[lane_id][12].y, top);
		//判断与当前车道是否相交
		dis_x = min(max(ptCorner[lane_id][1].x,ptCorner[lane_id][2].x), right) - max(min(ptCorner[lane_id][0].x,ptCorner[lane_id][3].x), left);
		dis_y = min(max(ptCorner[lane_id][2].y,ptCorner[lane_id][3].y), bottom) - max(min(ptCorner[lane_id][0].y,ptCorner[lane_id][1].y), top);
		//当目标在视频存在时间太长或长时间没有检测到或离开图像，删除目标
		bool isLost = FALSE;
		if(strcmp(pCfgs->targets[i].names, "bus") == 0 ||  strcmp(pCfgs->targets[i].names, "truck") == 0)//公交车或卡车
		{
			isLost = (pCfgs->targets[i].lost_detected > 10);
		}
		else
		{
			isLost = (pCfgs->targets[i].lost_detected > 5);
		}
		if(pCfgs->targets[i].continue_num > 5000 /*|| (pCfgs->targets[i].lost_detected > 5 && dis <= 0)*/ || isLost/*pCfgs->targets[i].lost_detected > 5*/ || dis_x < 0 || dis_y < 0)//10
		//if(pCfgs->targets[i].continue_num > 5000 || (pCfgs->targets[i].lost_detected > 5 && pCfgs->targets[i].box.y > ptCorner[lane_id][14].y)||(pCfgs->targets[i].lost_detected > 10 && (pCfgs->targets[i].box.y> ptCorner[lane_id][12].y))|| (pCfgs->targets[i].box.x < 0 || pCfgs->targets[i].box.y < 0 || (pCfgs->targets[i].box.x + pCfgs->targets[i].box.width) > imgW  ||((pCfgs->targets[i].box.y + pCfgs->targets[i].box.height) > imgH)))
		{
			if(pCfgs->currFore_target_id[lane_id] == pCfgs->targets[i].target_id && pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[0].calarflag == 1)//线圈是红色
				obj_out_region(pCfgs, lane_id, &pCfgs->targets[i], 0);//车出
			obj_lost[lane_id] = TRUE;
			DeleteTarget(&pCfgs->targets_size, &i, pCfgs->targets);
			continue;

		}
		//判断与当前车道流量线圈是否相交
		dis_x = min(max(ptCorner[lane_id][13].x,ptCorner[lane_id][14].x) + 10, right) - max(min(ptCorner[lane_id][12].x,ptCorner[lane_id][15].x) - 10, left);
		dis_y = min(max(ptCorner[lane_id][14].y,ptCorner[lane_id][15].y) + 10, bottom) - max(min(ptCorner[lane_id][12].y,ptCorner[lane_id][13].y) - 10, top);
		//去除不在检测区域的目标
		if(dis_x > 0 && dis_y > 0)
		{
			laneStatus[lane_id][0] = 1;
		}
		else if(pCfgs->targets[i].continue_num > 3)
		{
			if(pCfgs->currFore_target_id[lane_id] == pCfgs->targets[i].target_id && pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[0].calarflag == 1)//线圈是红色出车
				obj_out_region(pCfgs, lane_id, &pCfgs->targets[i], 0);//车出
			obj_lost[lane_id] = TRUE;
			//DeleteTarget(&pCfgs->targets_size, &i, pCfgs->targets);//这里删除目标，容易导致跳动框删除目标，然后多计数
			//continue;
		}
#ifdef SAVE_VIDEO
		cv::rectangle(img, cv::Rect(pCfgs->targets[i].box.x,pCfgs->targets[i].box.y,pCfgs->targets[i].box.width,pCfgs->targets[i].box.height), cv::Scalar(255, 255 ,255), 1, 8, 0 );
		char str[10];
		sprintf(str, "%d", pCfgs->targets[i].target_id);
		putText(img, str, cv::Point(pCfgs->targets[i].box.x,max(0,pCfgs->targets[i].box.y - 10)), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 2);
		char str1[10];
		sprintf(str1, "%d", pCfgs->currFore_target_id[lane_id]);
		putText(img, str1, cv::Point(pCfgs->targets[i].box.x + 30,max(0,pCfgs->targets[i].box.y - 10)), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 2);
#endif
		pCfgs->targets[i].continue_num++;

	}

	//分析检测目标，并进行区域内的车辆数统计
	for(i = 0;i < pCfgs->detTargets_size; i++)
	{
		//保存轨迹
		pCfgs->detTargets[i].trajectory[pCfgs->detTargets[i].trajectory_num].x = pCfgs->detTargets[i].box.x + pCfgs->detTargets[i].box.width / 2;
		pCfgs->detTargets[i].trajectory[pCfgs->detTargets[i].trajectory_num].y = pCfgs->detTargets[i].box.y + pCfgs->detTargets[i].box.height / 2;
		pCfgs->detTargets[i].trajectory_num++;

		//检测到，并更新速度
		if(pCfgs->detTargets[i].detected)
		{
			pCfgs->detTargets[i].lost_detected = 0;
			//get_speed(&pCfgs->detTargets[i]);
		}
		else//未检测到
		{
			pCfgs->detTargets[i].lost_detected++;
			pCfgs->detTargets[i].box.x += pCfgs->detTargets[i].vx;
			pCfgs->detTargets[i].box.y += pCfgs->detTargets[i].vy;
		}

		lane_id = pCfgs->detTargets[i].lane_id;
		left = pCfgs->detTargets[i].box.x;
		right = pCfgs->detTargets[i].box.x + pCfgs->detTargets[i].box.width;
		top = pCfgs->detTargets[i].box.y;
		bottom = pCfgs->detTargets[i].box.y + pCfgs->detTargets[i].box.height;
		dis_x = min(max(ptCorner[lane_id][9].x,ptCorner[lane_id][10].x) + 5, right) - max(min(ptCorner[lane_id][8].x,ptCorner[lane_id][11].x) - 5, left);
		dis_y = min(max(ptCorner[lane_id][10].y,ptCorner[lane_id][11].y) + 5, bottom) - max(min(ptCorner[lane_id][8].y,ptCorner[lane_id][9].y) - 5, top);
		//判断此目标是否进入占位线圈
		if(dis_x > 10 && dis_y > 10 && pCfgs->detTargets[i].target_id != pCfgs->currRear_target_id[lane_id] && pCfgs->detTargets[i].cal_flow == FALSE)
		{
			//满足时间间隔条件，进行判断
			if((pCfgs->gThisFrameTime - pCfgs->uRearIntervalNum[lane_id]) > 5 || pCfgs->uRearIntervalNum[lane_id] == 0)
			{
				//线圈已经有车辆存在
				if(pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[1].calarflag)
				{
					for( k = 0; k < pCfgs->detTargets_size; k++)
					{
						if(pCfgs->detTargets[k].target_id == pCfgs->currRear_target_id[lane_id])
						{
							break;
						}
					}
					//存在车辆
					if(k < pCfgs->detTargets_size)//先出车，再进车
					{
						obj_out_region(pCfgs, lane_id, &pCfgs->detTargets[k], 1);//后线圈出车
					}
					if(k == pCfgs->detTargets_size)
					{
						pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[1].calarflag = 2;
					}
				}
				//printf("lane_id = %d, intervalnum = %d,calarflag = %d,curr_id = %d,%d\n",lane_id,pCfgs->uRearIntervalNum[lane_id],pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[1].calarflag, pCfgs->detTargets[k].target_id,pCfgs->detTargets[i].target_id);
				pCfgs->currRear_target_id[lane_id] = pCfgs->detTargets[i].target_id;//赋予新的target_id
				pCfgs->uRearIntervalNum[lane_id] = pCfgs->gThisFrameTime;

			}
		}

		if(pCfgs->detTargets[i].target_id == pCfgs->currRear_target_id[lane_id] && pCfgs->detTargets[i].cal_flow == FALSE)
		{
			obj_in_region(pCfgs, lane_id, 1);//后线圈车入
			pCfgs->detTargets[i].cal_flow = TRUE;
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[1].uVehicleType = pCfgs->detTargets[i].class_id + 1;
		}

		//判断与当前车道占位线圈是否相交
		if(dis_x > 0 && dis_y > 0)//与线圈相交
		{
			laneStatus[lane_id][1] = 1;
/*#ifdef SAVE_VIDEO
			cv::rectangle(img, cv::Rect(left,top,pCfgs->detTargets[i].box.width,pCfgs->detTargets[i].box.height), cv::Scalar(255, 255 ,255), 1, 8, 0 );
			char str1[10];
			sprintf(str1, "%d", pCfgs->detTargets[i].target_id);
			putText(img, str1, cv::Point(left + 20,max(0,top - 10)), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 2);
#endif*/

		}
		else//与线圈不相交
		{
			if(pCfgs->detTargets[i].target_id == pCfgs->currRear_target_id[lane_id] && pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[1].calarflag == 1)//线圈是红色
			{
				obj_out_region(pCfgs, lane_id, &pCfgs->detTargets[i], 1);//后线圈出车
			}
/*#ifdef SAVE_VIDEO
			char str[10];
			sprintf(str, "%d", pCfgs->currRear_target_id[lane_id]);
			putText(img, str, cv::Point(left,max(0,top - 10)), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 2);
			cv::rectangle(img, cv::Rect(left,top,pCfgs->detTargets[i].box.width,pCfgs->detTargets[i].box.height), cv::Scalar(128, 128 ,0), 1, 8, 0 );
#endif*/

		}

		if(pCfgs->detTargets[i].target_id == pCfgs->currRear_target_id[lane_id] && pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[1].calarflag == 1)
		{
			//计算后线圈的流量、车速、车长
			dis = min(ptCorner[lane_id][10].y, bottom) - max(ptCorner[lane_id][8].y, top);
			if(dis >= min(pCfgs->detTargets[i].box.height, ptCorner[lane_id][10].y - ptCorner[lane_id][8].y - 2) && pCfgs->detTargets[i].cal_speed == FALSE && pCfgs->detTargets[i].continue_num)//当车辆到达流量区域下边缘
			{
			    pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[1].DetectOutSum++;
				//pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[1].uVehicleType = pCfgs->detTargets[i].class_id + 1;
				pCfgs->detTargets[i].end_time = pCfgs->currIime;
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[1].uVehicleSpeed  = CalTargetSpeed(pCfgs->detTargets[i], lane_id, pCfgs);
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[1].uVehicleLength = CalTargetLength(pCfgs->detTargets[i], lane_id, pCfgs);
				pCfgs->detTargets[i].cal_speed = TRUE;
			}
		}

		lane_id = pCfgs->detTargets[i].lane_id;

		//防止框跳动，出车
		dis_x = min(max(ptCorner[lane_id][2].x, ptCorner[lane_id][5].x), right) - max(min(ptCorner[lane_id][3].x, ptCorner[lane_id][4].x), left);
		dis_y = min(max(ptCorner[lane_id][2].y, ptCorner[lane_id][3].y), bottom) - max(min(ptCorner[lane_id][4].y, ptCorner[lane_id][5].y), top);
		//去除不在检测区域的目标
		if(dis_x > 0 && dis_y > 0)
		{
			//计算区域内的车辆数
			vehicle_num[lane_id]++;
			//将跟踪框代替检测框
			//pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1++] = pCfgs->detTargets[i].box;
			if(pCfgs->detTargets[i].detected)
			{
				pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].x = pCfgs->detTargets[i].box.x;
				pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].y = pCfgs->detTargets[i].box.y;
				pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].w = pCfgs->detTargets[i].box.width;
				pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].h = pCfgs->detTargets[i].box.height;
				pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].label = pCfgs->detTargets[i].class_id + 1;
				pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].confidence = pCfgs->detTargets[i].prob;
				pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].id = pCfgs->detTargets[i].target_id;//目标ID
				int pos_x = max(0, pCfgs->detTargets[i].box.x + pCfgs->detTargets[i].box.width / 2);
				pos_x = min(pos_x, imgW - 1);
				int pos_y =max(0, pCfgs->detTargets[i].box.y + pCfgs->detTargets[i].box.height / 2);
				pos_y = min(pos_y, imgH - 1);
				pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].distance = pCfgs->image_actual[pos_y][pos_x][1];//目标与相机的垂直距离
				pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].laneid = pCfgs->detTargets[i].lane_id;//车道号
				/*if(pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].w <= 0 || pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].h <= 0 || pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].w > imgW || pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].h > imgH)
				{
					prt(info," box width = %d, height = %d",pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].w, pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].h);
				}*/
				prt(info," box width = %d, height = %d",pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].w, pCfgs->ResultMsg.uResultInfo.udetBox[nboxes1].h);
				nboxes1++;
			}
		}
		else if(pCfgs->detTargets[i].continue_num > 5)
		{
			if(pCfgs->detTargets[i].target_id == pCfgs->currRear_target_id[lane_id] && pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[1].calarflag == 1)//线圈是红色
			{
				obj_out_region(pCfgs, lane_id, &pCfgs->detTargets[i], 1);//后线圈出车
			}
			DeleteTarget(&pCfgs->detTargets_size, &i, pCfgs->detTargets);
			continue;
		}
		//当目标在视频存在时间太长或长时间没有检测到或离开图像，删除目标
		if(pCfgs->detTargets[i].continue_num > 5000 || pCfgs->detTargets[i].lost_detected > 5 ||(pCfgs->detTargets[i].lost_detected > 0 && (left < 10 || top < 10 || right >= (imgW - 10)  ||bottom >= (imgH - 10))))
		{
			if(pCfgs->detTargets[i].target_id == pCfgs->currRear_target_id[lane_id] && pCfgs->ResultMsg.uResultInfo.uEachLaneData[lane_id].SpeedDetectInfo1.CoilAttribute[1].calarflag == 1)//线圈是红色
			{
				obj_out_region(pCfgs, lane_id, &pCfgs->detTargets[i], 1);//后线圈出车
			}
			DeleteTarget(&pCfgs->detTargets_size, &i, pCfgs->detTargets);
			continue;

		}
		pCfgs->detTargets[i].continue_num++;

	}
	pCfgs->ResultMsg.uResultInfo.udetNum = nboxes1; 

	//防止车辆跳变对区域车辆数进行处理
	for( i = 0; i < laneNum; i++)
	{
		//判断是否与线圈相交
		for(j = 0; j < 2; j++)
		{
			if(pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].calarflag == 1 && laneStatus[i][j] == 0 && pCfgs->existFrameNum[i][j] > 3)//没有目标在线圈内
			{
				if(j == 0)//流量线圈
				{
					for(k = 0; k < pCfgs->targets_size; k++)
					{
						if(pCfgs->currFore_target_id[i] == pCfgs->targets[k].target_id)
							obj_out_region(pCfgs, i, &pCfgs->targets[k], 0);
					}
					if(k == pCfgs->targets_size)//没有目标
					{
						if(pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[0].calarflag == 1)//车出线圈时，calarflag设置为2
							pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[0].calarflag = 2;
					}
				}
				if(j == 1)//后线圈
				{
					for(k = 0; k < pCfgs->detTargets_size; k++)
					{
						if(pCfgs->currRear_target_id[i] == pCfgs->detTargets[k].target_id)
							obj_out_region(pCfgs, i, &pCfgs->detTargets[k], 1);
					}
					if(k == pCfgs->detTargets_size)//没有目标
					{
						if(pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[1].calarflag == 1)//车出线圈时，calarflag设置为2
							pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[1].calarflag = 2;
					}

				}
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].calarflag = 2;
				pCfgs->existFrameNum[i][j] = 0;
			}
		}
		//printf("laneID = %d, vehicle num = %d\n",i,vehicle_num1[i]);
		if(pCfgs->gThisFrameTime < 10)//对最初几帧不进行处理
		{
			pCfgs->uDetectVehicleSum[i] = vehicle_num[i];
		}
		else
		{
			int curr_num = vehicle_num[i];
			//printf("lane id = %d,curr num = %d,old num =%d\n",i,curr_num,pCfgs->uDetectVehicleSum[i]);
			if(pCfgs->uDetectVehicleSum[i] != curr_num)
			{
				if(pCfgs->uDetectVehicleFrameNum[i] > 10)//最短10帧才发生车辆数的变化
				{
					curr_num = (pCfgs->uDetectVehicleSum[i] > curr_num)? (pCfgs->uDetectVehicleSum[i] - 1) : (pCfgs->uDetectVehicleSum[i] + 1);
				}
				else if(pCfgs->uDetectVehicleSum[i] >  curr_num && obj_lost[i])//有车出
				{
					curr_num = pCfgs->uDetectVehicleSum[i] - 1;
				}
				else
				{
					curr_num = pCfgs->uDetectVehicleSum[i];
				}
				if(vehicle_num[i] == 0 && curr_num != 0)//当前帧没有区域车辆数，快速将车辆数降下
				{
					if(pCfgs->uDetectVehicleFrameNum[i] > 5)
						curr_num = pCfgs->uDetectVehicleSum[i] - 1;
				}
			}
			//printf("lane id1 = %d,curr num = %d,old num =%d interval_num =%d\n",i,curr_num,pCfgs->uDetectVehicleSum[i],pCfgs->uDetectVehicleFrameNum[i]);
			if(pCfgs->uDetectVehicleSum[i] != curr_num)//前后两帧的车辆数不相同时，重新计数
				pCfgs->uDetectVehicleFrameNum[i] = 0;
			if(pCfgs->uDetectVehicleSum[i] == curr_num)//前后两帧车辆数相同
				pCfgs->uDetectVehicleFrameNum[i]++;
			pCfgs->uDetectVehicleSum[i] = curr_num;//将处理后的数量赋给当前车辆数
		}
		if(pCfgs->uDetectVehicleSum[i] == 0 || vehicle_num1[i] == 0)//车道区域内无车
		{
			pCfgs->Tailposition[i] = min(ptCorner[i][2].y, ptCorner[i][3].y);
			pCfgs->Headposition[i] = min(ptCorner[i][2].y, ptCorner[i][3].y);
		}
	}

	/*for( i = 0; i < laneNum; i++)
	{
		for( j = 1;j >= 0; j--)
		{
			pCfgs->uStatVehicleSum[i][j + 1] = pCfgs->uStatVehicleSum[i][j];
		}
		pCfgs->uStatVehicleSum[i][0] = vehicle_num[i];
		if(pCfgs->uStatVehicleSum[i][3] < 3)
			pCfgs->uStatVehicleSum[i][3] = pCfgs->uStatVehicleSum[i][3] + 1;

		sum = 0.0;
		for(j = 0; j < pCfgs->uStatVehicleSum[i][3];j++)
			sum +=  pCfgs->uStatVehicleSum[i][j];
		sum = sum / pCfgs->uStatVehicleSum[i][3];
		//防止区域车辆数跳变
		if(pCfgs->uDetectVehicleSum[i] > vehicle_num[i] && obj_lost[i])//有车出，数量减1
		{
			pCfgs->uDetectVehicleSum[i] = pCfgs->uDetectVehicleSum[i] - 1;
		} 
		else if(vehicle_num[i] > pCfgs->uDetectVehicleSum[i])
		{
			//pCfgs->uDetectVehicleSum[i] = vehicle_num[i];
			pCfgs->uDetectVehicleSum[i] = sum + 0.5;
		} 
		else if(pCfgs->uDetectVehicleSum[i] > vehicle_num[i] && pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[0].calarflag == 0)//流量区域无车，采用实际检测数
		{
			//pCfgs->uDetectVehicleSum[i] = vehicle_num[i];//实际车辆数
			//防止车辆数跳变
			pCfgs->uDetectVehicleSum[i] = sum + 0.5;
		}
		if(vehicle_num[i] == 0)//车道区域内无车
		{
		pCfgs->Tailposition[i] = min(ptCorner[i][2].y, ptCorner[i][3].y);
		pCfgs->Headposition[i] = min(ptCorner[i][2].y, ptCorner[i][3].y);
		}
#ifdef SAVE_VIDEO
		char str[10];
		sprintf(str, "%d", pCfgs->uDetectVehicleSum[i]);
		putText(img, str, Point(10 + 30 * i,10), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 2);
		char str1[10];
		sprintf(str1, "%d", pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[0].car_out);
		putText(img, str1, Point(10 + 30 * i,30), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 2);
#endif
	}*/
	//printf("get_target end...................................\n");
}
Uint16 ArithProc(Uint16 ChNum, unsigned char *pInFrameBuf, unsigned char *pInuBuf, unsigned char *pInvBuf, int FrameWidth, int FrameHeight,\
	RESULTMSG* outBuf, Int32 outSize, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	Int32 i, j;
	bool	bSpeedEnable, bPresenceEnable;
	float temp=0;

	CPoint m_ptend[16];
	CPoint LineUp[2];
	CPoint LineUp1[2];
	int x1 = 0, x2 = 0, x3 = 0, x4 = 0;
	int thr=10;

	int result[6 * MAX_DETECTION_NUM] = { 0 };
	Uint16 nboxes = 0;

	//将图像限制在[0, FULL_COLS),[0, FULL_ROWS)
	if(FrameWidth <= 0 || FrameHeight <= 0)//没有图像数据
	{
		printf("img cannot be zero!\n");
		return 0;
	}
	//处理数据大小
	pCfgs->m_iHeight = (FrameHeight > FULL_ROWS)? FULL_ROWS : FrameHeight;
	pCfgs->m_iWidth = (FrameWidth > FULL_COLS)? FULL_COLS : FrameWidth;

	if((pCfgs->bAuto == 1)&&pCfgs->gThisFrameTime%199==1&&pCfgs->targets_size<10)//daytime
	//if((pCfgs->CameraCfg.uEnvironmentStatus==4)&&pCfgs->gThisFrameTime%199==1&&pCfgs->targets_size<10)//daytime
	{
		//能见度计算
		thr=8;
		pCfgs->up_visib_value++;
		pCfgs->fuzzydegree=fuzzy(pInFrameBuf, FrameWidth, FrameHeight);
		for (j=VISIB_LENGTH-1;j>0;j--)
		{
			pCfgs->visib_value[j]=pCfgs->visib_value[j-1];
		}
		pCfgs->visib_value[0]=(int)(pCfgs->fuzzydegree);
		if (pCfgs->up_visib_value>VISIB_LENGTH)
		{
			pCfgs->visibility=visible_judge(pCfgs->visib_value,VISIB_LENGTH,thr);
		} 
		else
		{
			pCfgs->visibility=FALSE;
		}
		printf("fuzzy degree = %d\n",(int)(pCfgs->fuzzydegree));
		//视频异常计算
		if(Color_deviate(pInuBuf, pInvBuf, FrameWidth / 4, FrameHeight / 4))
			pCfgs->abnormal_time++;
		else
			pCfgs->abnormal_time = 0;
		pCfgs->fuzzyflag = (pCfgs->abnormal_time > 5)? TRUE : FALSE;
	}
	else if(pCfgs->bAuto == 2)//night
	//else if(pCfgs->CameraCfg.uEnvironmentStatus==2||pCfgs->CameraCfg.uEnvironmentStatus==1||pCfgs->CameraCfg.uEnvironmentStatus==3)//night dawn 
	{
		pCfgs->visibility = FALSE;
		pCfgs->fuzzyflag = FALSE;
	}

	//得到车道区域内的像素值
	//StoreCurImage((Uint8 *)inBufs->bufDesc[0].buf, pCfgs, pParams);
	//printf("StoreCurImage is ok");

	//设置车道掩模图像
	if(pCfgs->bMaskLaneImage == FALSE)
	{
		MaskLaneImage(pCfgs, pParams, FrameWidth, FrameHeight);
		pCfgs->bMaskLaneImage = TRUE;
	}
	//设置行人检测区域掩模图像
	if(pCfgs->bMaskDetectImage == FALSE)
	{
		MaskDetectImage(pCfgs, pParams, FrameWidth, FrameHeight);
		pCfgs->bMaskDetectImage = TRUE;
	}
	//标定图像
	if(pCfgs->bCalibrationImage == FALSE)
	{
		get_calibration_data(pCfgs, FrameWidth, FrameHeight);
		pCfgs->bCalibrationImage = TRUE;
	}
	gettimeofday(&pCfgs->time_end, NULL);
	if(pCfgs->gThisFrameTime == 0)
		pCfgs->currIime = 0;
	else
	pCfgs->currIime += (pCfgs->time_end.tv_sec - pCfgs->time_start.tv_sec) + (pCfgs->time_end.tv_usec - pCfgs->time_start.tv_usec)/1000000.0;
	gettimeofday(&pCfgs->time_start, NULL);

	pCfgs->gThisFrameTime++;
	////////////////////////////////////////////////
	//将yuv转化成bgr
	Mat YUVImage, BGRImage;
	int size = FrameWidth * FrameHeight;
	//yuv420 to bgr
	YUVImage.create(FrameHeight * 3 / 2, FrameWidth, CV_8UC1);
	memcpy(YUVImage.data, pInFrameBuf, size);
	memcpy(YUVImage.data + size, pInuBuf, size /4);
	memcpy(YUVImage.data + size + size /4, pInvBuf, size / 4);
	cvtColor(YUVImage, BGRImage, CV_YUV2BGR_I420);
	YUVImage.release();

	//进行目标检测
	nboxes = ArithDetect(pCfgs, BGRImage, result);
#ifdef DETECT_PLATE//车牌识别
	PlateInfo plateInfo[MAX_PLATE_NUM];
	if(pCfgs->plate_flag >= 0)
	{
		int plateNum = PlateDetectandRecognize(BGRImage.data, FrameWidth, FrameHeight, 0, plateInfo, pCfgs->plate_flag);//车牌识别
		printf("plate num = %d\n",plateNum);
	}
#endif

#ifdef SAVE_VIDEO
	BGRImage.copyTo(img);
#endif
	//////////////////////////////////////////////////////
	memset((void *)&pCfgs->ResultMsg, 0, sizeof(pCfgs->ResultMsg));
	memcpy((void *)&pCfgs->ResultMsg, (void *)outBuf, outSize);
	bSpeedEnable = (bool)(pCfgs->CameraLocalPara.bNormalDetectEnable & 0x00000001);
	bPresenceEnable = (bool)(pCfgs->CameraLocalPara.bNormalDetectEnable & 0x00000002);
	//分析检测结果
	get_target(BGRImage, result, nboxes, pCfgs, pParams, pCfgs->CameraCfg.LaneAmount, FrameWidth, FrameHeight);
	pCfgs->ResultMsg.uResultInfo.LaneSum = pCfgs->CameraCfg.LaneAmount;//车道数

	//缩放图像
	Mat grayImage, resizeImage;
	grayImage.create(FrameHeight, FrameWidth, CV_8UC1);
	memcpy(grayImage.data, pInFrameBuf, FrameHeight * FrameWidth);
	if(pCfgs->m_iWidth != FrameWidth || pCfgs->m_iHeight != FrameHeight)
		resize(grayImage, resizeImage, Size(pCfgs->m_iWidth, pCfgs->m_iHeight), 0, 0, INTER_LINEAR);//缩放
	else
		grayImage.copyTo(resizeImage);//复制
	grayImage.release();
	memcpy((void *)pParams->CurrQueueImage, (void *)resizeImage.data, pCfgs->m_iWidth * pCfgs->m_iHeight);
	//计算车道内的排队长度
	for( i = 0; i < pCfgs->CameraCfg.LaneAmount; i++ )
	{

		if(bSpeedEnable == TRUE)
		{
			m_ptend[0]= pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[0]; 
			m_ptend[1]= pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[1];
			m_ptend[2]= pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[2];  
			m_ptend[3]= pCfgs->DownSpeedCfg.SpeedEachLane[i].LaneRegion[3];//车道区域
			m_ptend[4]= pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[0];  
			m_ptend[5]= pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[1];
			m_ptend[6]= pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[2];  
			m_ptend[7]= pCfgs->DownSpeedCfg.SpeedEachLane[i].FrontCoil[3]; //占位线圈		
			m_ptend[8]= pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[0];  
			m_ptend[9]= pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[1];
			m_ptend[10]= pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[2];  
			m_ptend[11]= pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[3];//占有线圈
			m_ptend[12]= pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[0];  
			m_ptend[13]= pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[1];
			m_ptend[14]= pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[2];  
			m_ptend[15]= pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[3];//流量线圈
#ifdef DETECT_REAR
			SpeedCaculate( i, pCfgs, pParams, m_ptend, FrameWidth, FrameHeight, 1);
#else
			SpeedCaculate( i, pCfgs, pParams, m_ptend, FrameWidth, FrameHeight, 0);
#endif
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.getQueback_flag=1;
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.IsCarInTailFlag = pCfgs->IsCarInTail[i];
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uDetectRegionVehiSum = pCfgs->uDetectVehicleSum[i];


			LineUp[0].x = LineUp[0].y = LineUp[1].x = LineUp[1].y = 0;
			LineUp1[0].x = LineUp1[0].y =LineUp1[1].x = LineUp1[1].y = 0;
			//末车位置

			if(m_ptend[0].y != m_ptend[2].y && m_ptend[1].y != m_ptend[3].y)
			{
				LineUp[0].y = pCfgs->Headposition[i];
				LineUp[1].y = pCfgs->Headposition[i];
				if(m_ptend[0].x == m_ptend[3].x)//垂直车道线
				{
					LineUp[0].x = m_ptend[0].x;
				}
				else
				{
					LineUp[0].x = (LineUp[0].y - m_ptend[0].y) * (m_ptend[3].x - m_ptend[0].x) / (m_ptend[3].y - m_ptend[0].y) + m_ptend[0].x;
				}
				if(m_ptend[1].x == m_ptend[2].x)//垂直车道线
				{
					LineUp[1].x = m_ptend[1].x;
				}	
				else
				{
					LineUp[1].x = (LineUp[1].y - m_ptend[1].y) * (m_ptend[2].x - m_ptend[1].x) / (m_ptend[2].y - m_ptend[1].y) + m_ptend[1].x;
				}
			}
			else
			{
				printf("detect point err\n");
			}
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.LineUp[0] = LineUp[0];
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.LineUp[1] = LineUp[1];

			//头车位置、末车位置、 头车速度、末车速度
			if(pCfgs->uDetectVehicleSum[i] == 0)
			{
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uHeadVehiclePos = 0;//头车位置
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uLastVehiclePos = 0;//末车位置
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uHeadVehicleSpeed = 0;//头车速度
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uLastVehicleSpeed = 0;//末车速度
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uVehicleDensity = 0;//车辆密度
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uLastVehicleLength = 0;//最后一辆车位置
			}
			else
			{	
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uHeadVehiclePos = pCfgs->actual_distance[i][pCfgs->Tailposition[i]];//头车位置
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uLastVehiclePos =pCfgs->actual_distance[i][pCfgs->Headposition[i]];//末车位置
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uHeadVehicleSpeed = pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[0].uVehicleSpeed;//头车速度
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uLastVehicleSpeed = pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[1].uVehicleSpeed;//末车速度
				//最后一辆车的位置
				temp = pCfgs->actual_distance[i][pCfgs->Headposition[i]];
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uLastVehicleLength = temp;

			}
			
			//排队长度
			if(m_ptend[0].y != m_ptend[3].y && m_ptend[1].y != m_ptend[2].y)
			{
				LineUp1[0].y = pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uQueueHeadDis;//队首
				LineUp1[1].y = pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uQueueTailDis;//队尾
				if(m_ptend[0].x == m_ptend[3].x)//垂直车道线
				{
					x1 = m_ptend[0].x;
					x3 = m_ptend[0].x;
				}
				else
				{
					x1 = (LineUp1[1].y - m_ptend[0].y) * (m_ptend[3].x - m_ptend[0].x) / (m_ptend[3].y - m_ptend[0].y) + m_ptend[0].x;
					x3 = (LineUp1[0].y - m_ptend[0].y) * (m_ptend[3].x - m_ptend[0].x) / (m_ptend[3].y - m_ptend[0].y) + m_ptend[0].x;
				}
				if(m_ptend[1].x == m_ptend[2].x)//垂直车道线
				{
					x2 = m_ptend[1].x;
					x4 = m_ptend[1].x;
				}	
				else
				{
					x2 = (LineUp1[1].y - m_ptend[1].y) * (m_ptend[2].x - m_ptend[1].x) / (m_ptend[2].y - m_ptend[1].y) + m_ptend[1].x;
					x4 = (LineUp1[0].y - m_ptend[1].y) * (m_ptend[2].x - m_ptend[1].x) / (m_ptend[2].y - m_ptend[1].y) + m_ptend[1].x;
				}
				LineUp1[1].x = (x1 + x2) / 2;
				LineUp1[0].x = (x3 + x4) / 2;
			}
			else
			{
				printf("detect point err\n");
			} 
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.QueLine[0] = LineUp1[0];//队首
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.QueLine[1] = LineUp1[1];//队尾
			if(abs(LineUp1[0].y - LineUp1[1].y) < 5)//没有排队	
			{	
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uQueueHeadDis = 0;//队首距离
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uQueueTailDis = 0;//队尾距离
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uVehicleQueueLength = 0;//排队长度

			}
			else
			{
				temp = pCfgs->actual_distance[i][LineUp1[0].y];	
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uQueueHeadDis = temp;//队首距离
				temp = pCfgs->actual_distance[i][LineUp1[1].y];	
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uQueueTailDis = temp;//队尾距离
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uVehicleQueueLength = pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uQueueTailDis;//从停车线开始
				//pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uVehicleQueueLength = abs(pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uQueueTailDis - pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uQueueHeadDis);//队首减队尾  排队长度
			}
#ifdef DETECT_REAR
			if(LineUp1[1].y < min(m_ptend[12].y, m_ptend[13].y))//排队长度没有超过流量区域，则认为不是排队
			{
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uVehicleQueueLength = 0;
			}
#endif
#ifdef SAVE_VIDEO
			cv::line(img, cv::Point(pCfgs->detLineParm.pt[0].x,pCfgs->detLineParm.pt[0].y),cv::Point(pCfgs->detLineParm.pt[1].x,pCfgs->detLineParm.pt[1].y), cv::Scalar(255, 0 ,0), 1, 8, 0 );
			cv::line(img, cv::Point(m_ptend[0].x,m_ptend[0].y),cv::Point(m_ptend[3].x,m_ptend[3].y), cv::Scalar(255, 0 ,0), 1, 8, 0 );
			cv::line(img, cv::Point(m_ptend[1].x,m_ptend[1].y),cv::Point(m_ptend[2].x,m_ptend[2].y), cv::Scalar(255, 0 ,0), 1, 8, 0 );
			if(pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[0].calarflag)
			{
				cv::line(img, cv::Point(m_ptend[12].x,m_ptend[12].y),cv::Point(m_ptend[13].x,m_ptend[13].y), cv::Scalar(0, 0 ,255), 1, 8, 0 );
				cv::line(img, cv::Point(m_ptend[14].x,m_ptend[14].y),cv::Point(m_ptend[15].x,m_ptend[15].y), cv::Scalar(0, 0 ,255), 1, 8, 0 );
			}
			else
			{
				cv::line(img, cv::Point(m_ptend[12].x,m_ptend[12].y),cv::Point(m_ptend[13].x,m_ptend[13].y), cv::Scalar(255, 0 ,0), 1, 8, 0 );
				cv::line(img, cv::Point(m_ptend[14].x,m_ptend[14].y),cv::Point(m_ptend[15].x,m_ptend[15].y), cv::Scalar(255, 0 ,0), 1, 8, 0 );
			}
			if(pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[1].calarflag)
			{
				cv::line(img, cv::Point(m_ptend[8].x,m_ptend[8].y),cv::Point(m_ptend[9].x,m_ptend[9].y), cv::Scalar(0, 0 ,255), 1, 8, 0 );
				cv::line(img, cv::Point(m_ptend[10].x,m_ptend[10].y),cv::Point(m_ptend[11].x,m_ptend[11].y), cv::Scalar(0, 0 ,255), 1, 8, 0 );
			}
			else
			{
				cv::line(img, cv::Point(m_ptend[8].x,m_ptend[8].y),cv::Point(m_ptend[9].x,m_ptend[9].y), cv::Scalar(255, 0 ,0), 1, 8, 0 );
				cv::line(img, cv::Point(m_ptend[10].x,m_ptend[10].y),cv::Point(m_ptend[11].x,m_ptend[11].y), cv::Scalar(255, 0 ,0), 1, 8, 0 );
			}

			cv::line(img, cv::Point(pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.QueLine[0].x,pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.QueLine[0].y),cv::Point(pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.QueLine[1].x,pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.QueLine[1].y), cv::Scalar(0, 255 ,0), 3, 8, 0 );
			//cv::line(img, cv::Point(pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.LineUp[0].x,pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.LineUp[0].y),cv::Point(pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.LineUp[1].x,pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.LineUp[1].y), cv::Scalar(255, 0 ,0), 3, 8, 0 );
			if(pCfgs->IsCarInTail[i])
			{
				cv::line(img, cv::Point(m_ptend[4].x,m_ptend[4].y),cv::Point(m_ptend[5].x,m_ptend[5].y), cv::Scalar(0, 0 ,255), 1, 8, 0);
				cv::line(img, cv::Point(m_ptend[6].x,m_ptend[6].y),cv::Point(m_ptend[7].x,m_ptend[7].y), cv::Scalar(0, 0 ,255), 1, 8, 0);
			}
			else
			{
				cv::line(img, cv::Point(m_ptend[4].x,m_ptend[4].y),cv::Point(m_ptend[5].x,m_ptend[5].y), cv::Scalar(255, 0 ,0), 1, 8, 0);
				cv::line(img, cv::Point(m_ptend[6].x,m_ptend[6].y),cv::Point(m_ptend[7].x,m_ptend[7].y), cv::Scalar(255, 0 ,0), 1, 8, 0);
			}
			char str[10];
			sprintf(str, "%d", pCfgs->gThisFrameTime);
			putText(img, str, cv::Point(320,30), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 2);
			char str1[10];
			sprintf(str1, "%d", pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[0].DetectInSum);
			putText(img, str1, cv::Point(10 + 50 * i, 60), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 2);
			char str2[10];
			sprintf(str2, "%d", pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[0].DetectOutSum);
			putText(img, str2, cv::Point(10 + 50 * i, 90), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 2);
			char str3[10];
			sprintf(str3, "%d", pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[1].DetectInSum);
			putText(img, str3, cv::Point(10 + 50 * i, 180), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 2);
			char str4[10];
			sprintf(str4, "%d", pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[1].DetectOutSum);
			putText(img, str4, cv::Point(10 + 50 * i, 210), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 2);

#endif
		}
	}
	memcpy((void *)pParams->PrePrePreQueueImage, (void *)pParams->PrePreQueueImage, pCfgs->m_iWidth * pCfgs->m_iHeight);
	memcpy((void *)pParams->PrePreQueueImage, (void *)pParams->PreQueueImage, pCfgs->m_iWidth * pCfgs->m_iHeight);
	memcpy((void *)pParams->PreQueueImage, (void *)pParams->CurrQueueImage, pCfgs->m_iWidth * pCfgs->m_iHeight);
#ifdef SAVE_VIDEO
	 writer << img; 
	 if(pCfgs->gThisFrameTime > SAVE_FRAMES)
		 writer.release();
	 //imshow("img",img);
	 //waitKey(1);
#endif 
	BGRImage.release();
	if((pCfgs->CameraCfg.uEnvironmentStatus == 1) && (pCfgs->bAuto == 1))//黄昏
	{
		pCfgs->bAuto = 1;
		pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight = FALSE;
		pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bDay = TRUE;
		pCfgs->bNight = FALSE;
	}
	else if((pCfgs->CameraCfg.uEnvironmentStatus == 3) && (pCfgs->bAuto == 2))//黎明
	{
		pCfgs->bAuto = 2;
		pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight = TRUE ;
		pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bDay = FALSE ;
		pCfgs->bNight = TRUE;
	}
	else if (pCfgs->CameraCfg.uEnvironmentStatus == 2)//晚上
	{
		pCfgs->bAuto = 2;
		pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight = TRUE ;
		pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bDay = FALSE ;
		pCfgs->bNight = TRUE;
	}
	else if(pCfgs->CameraCfg.uEnvironmentStatus == 4)//白天
	{
		pCfgs->bAuto = 1;
		pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight = FALSE;
		pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bDay = TRUE;
		pCfgs->bNight = FALSE;
	}

	pCfgs->ResultMsg.uResultInfo.uEnvironmentStatus = pCfgs->bAuto; //added by david 20131014
	memcpy((void *)outBuf, (void *)&pCfgs->ResultMsg, outSize);
	return 1;
}
//按照位置顺序对检测框进行排序
void sort_obj(int obj_pos[][2], int obj_num)
{
	int temp[2];
	int i = 0, j = 0;
	for(i = 0; i < obj_num - 1; i++)

	{
		for(j = i + 1; j < obj_num; j++)

        {
			if(obj_pos[i][0] < obj_pos[j][0])
			{
				temp[0] = obj_pos[i][0];
				temp[1] = obj_pos[i][1];
				obj_pos[i][0] = obj_pos[j][0];
				obj_pos[i][1] = obj_pos[j][1];
				obj_pos[j][0] = temp[0];
				obj_pos[j][1] = temp[1];
			}
		}
	}

}
//是否排队
bool IsVehicleQueue(Uint8* puSubImage, Uint8* puMaskImage, Uint16 laneID, Uint16 tail_line, Uint16 head_line, ALGCFGS *pCfgs)
{
	int nRow,nCol;
	int subnum = 0, num = 0;
	float ratio = 0.0;
	Uint32 offset = 0;
	unsigned char* subPtr;
	unsigned char* maskPtr;
	//计算运动点数
	for( nRow =tail_line ; nRow < head_line; nRow++ )
	{
		offset = nRow * pCfgs->m_iWidth;
		subPtr = puSubImage + offset;
		maskPtr = puMaskImage + offset;
		for (nCol =0 ; nCol < pCfgs->m_iWidth; nCol++)
		{
			if(maskPtr[nCol] == (laneID + 1))
			{
				if (subPtr[nCol])
				{
					subnum++;
				}
				num++;
			}
		}
	}
	ratio = (num > 0)? (float)subnum / (float)num : 0;
	if(ratio < 0.15)//0.15  小于阈值，则认为此区域不运动
	{
		return true;
	} 
	else
	{
		return false;
	}
}
Uint16 sort_median(Uint16* arr, int num)//取中间值
{
	int i = 0, j = 0;
	Uint16 temp;
	Uint16 array_temp[10] = {0};
	for(i = 0; i < num; i++)
	{
		array_temp[i] = arr[i];
	}
	for(i = 0; i <= num / 2; i++)
	{
		for(j = i + 1; j < num; j++)
		{
			if(array_temp[i] > array_temp[j])
			{
				temp = array_temp[i];
				array_temp[i] = array_temp[j];
				array_temp[j] = temp;
			}
		}
	}
	return array_temp[num / 2];
}
//计算排队长度
void SpeedCaculate(Uint16 LaneID, ALGCFGS *pCfgs, ALGPARAMS	*pParams, CPoint m_ptend[], int width, int height, int flag)// 0 代表车头，1代表车尾
{
	int i = 0, j = 0;
	int top = 0, bottom = 0;
	int obj_pos[50][2];//车辆框位置
	int obj_num = 0;//车辆框数量
	//int lane_top = min(m_ptend[0].y, m_ptend[1].y);//车道最上端
	int lane_top = 0;
	int lane_bottom = min(m_ptend[2].y, m_ptend[3].y);//车道最下端
	int que_start_pos = 0, que_end_pos = 0, que_pos1 = 0;//排队开始位置、结束位置
	float obj_interval[50] = { 0 };//框间距
	int no_interval_num = 0;
	bool IsVehStatic[50]; //车辆位置是否静止
	float sum = 0.0;
	int stat_num = 0, num = 0;
	int QueVehicleNum = 0;//排队区域内车辆数量
	int actual_dis = 0;
	if(flag == 0)//车头
	{
		lane_top = min(m_ptend[0].y, m_ptend[1].y);//车道最上端
	}
	else//车尾
	{
		lane_top = min(m_ptend[4].y, m_ptend[5].y);//车道占有线圈的最上端
	}
	//根据运动情况和车辆数进行排队长度计算
	//iSubStractImage(pParams->CurrQueueImage, pParams->PreQueueImage, 15, 0, pCfgs->team_height, pCfgs->team_width, pCfgs->team_height);
	iSubStractImage(pParams->CurrQueueImage, pParams->PrePrePreQueueImage, pParams->MaskLaneImage, 15, LaneID, pCfgs->m_iWidth, pCfgs->m_iHeight);//隔三帧帧差
	/*//根据检测框进行排队长度分析
	for( i = 0; i < pCfgs->classes; i++)
	{
		if(pCfgs->detClasses[i].classes_num)
		{
			for( j = 0; j < pCfgs->detClasses[i].classes_num; j++)
			{
				if(pCfgs->detClasses[i].lane_id[j] == LaneID)
				{
					bottom = min(pCfgs->detClasses[i].box[j].y + pCfgs->detClasses[i].box[j].height, lane_bottom);
					top = min(pCfgs->detClasses[i].box[j].y, lane_bottom);
					if(bottom - top > 5)
					{
						if(obj_num < MAX_LANE_TARGET_NUM)
						{
							pCfgs->detBoxes[LaneID][obj_num] = pCfgs->detClasses[i].box[j];
						}//save
						obj_pos[obj_num][0] = bottom;
						obj_pos[obj_num][1] = top;
						obj_num++;
					}
				}
			}
		}
	}*/
	////根据跟踪框进行排队长度分析
	for( i = 0; i < pCfgs->detTargets_size; i++)
	{
		if(pCfgs->detTargets[i].lane_id == LaneID)
		{
			bottom = min(pCfgs->detTargets[i].box.y + pCfgs->detTargets[i].box.height, lane_bottom);
			top = min(pCfgs->detTargets[i].box.y, lane_bottom);
			if(bottom - top > 5)
			{
				if(obj_num < MAX_LANE_TARGET_NUM)
				{
					pCfgs->detBoxes[LaneID][obj_num] = pCfgs->detTargets[i].box;
				}//save
				obj_pos[obj_num][0] = bottom;
				obj_pos[obj_num][1] = top;
				obj_num++;
			}
		}
	}

	pCfgs->detNum[LaneID] = (obj_num > MAX_LANE_TARGET_NUM)? MAX_LANE_TARGET_NUM : obj_num;
	sort_obj(obj_pos, obj_num);//排序，从下向上
	for( i = 0; i < obj_num; i++)//计算每个框的间隔值和运动情况
	{
		if(i == 0)
		{
			obj_interval[i] = (float)(lane_bottom - obj_pos[i][0])/(float)(obj_pos[i][0] - obj_pos[i][1]);
			//sum += obj_interval[i];
			sum += (lane_bottom - obj_pos[i][0] < 0)? 0 : (lane_bottom - obj_pos[i][0]);//不是车辆的行数
		}
		if(i != 0)
		{
			obj_interval[i] = (float)(obj_pos[i - 1][1] - obj_pos[i][0])/(float)(obj_pos[i][0] - obj_pos[i][1]);
			//sum += obj_interval[i];
			if(obj_interval[i] <= 0)
				no_interval_num++;
			else
				no_interval_num = 0;
			sum += (obj_pos[i - 1][1] - obj_pos[i][0] < 0)? 0 :(obj_pos[i - 1][1] - obj_pos[i][0]);//不是车辆的行数
			actual_dis += (pCfgs->actual_distance[LaneID][obj_pos[i][0]] - pCfgs->actual_distance[LaneID][obj_pos[i - 1][1]]) * (pCfgs->actual_distance[LaneID][obj_pos[i][0]] - pCfgs->actual_distance[LaneID][obj_pos[i - 1][1]]);//车辆之间距离方差
		}
		if(i == obj_num - 1)
		{
			sum += (obj_pos[i][1] - lane_top < 0)? 0 :(obj_pos[i][1] - lane_top);//不是车辆的行数
		}
		IsVehStatic[i] = IsVehicleQueue(pParams->PrePrePreQueueImage, pParams->MaskLaneImage, LaneID, max(0, obj_pos[i][1] * pCfgs->m_iHeight / height), max(0, min(pCfgs->m_iHeight - 1, obj_pos[i][0] * pCfgs->m_iHeight / height)), pCfgs);
	}
	actual_dis = (obj_num > 1)? actual_dis / (obj_num - 1) : 0;

	//分析排队情况
	if(flag == 0)//车头
	{
		que_start_pos = lane_bottom;
		que_end_pos = lane_bottom;
		for( i = 0; i < obj_num; i++)
		{
			if(obj_interval[i] > 2)
			{
				break;
			}
			if(i == 1 && IsVehStatic[i] == FALSE && IsVehStatic[i - 1] == FALSE)
			{
				break;
			}
			if((IsVehStatic[i] && obj_interval[i] < 0.2 && (i == 0)) || (IsVehStatic[i] && IsVehStatic[i - 1] && obj_interval[i] < 0.5 && (i != 0)))// 0.1  0.5
			{
				que_end_pos = obj_pos[i][1];
			}
		}
	}
	else//车尾
	{
		que_start_pos = lane_top;
		que_end_pos = lane_top;
		for( i = obj_num - 1; i >= 0; i--)
		{
			if(obj_interval[i] > 2)
			{
				break;
			}
			if(i == obj_num - 2 && IsVehStatic[i] == FALSE && IsVehStatic[i + 1] == FALSE)
			{
				break;
			}
			if((IsVehStatic[i] && obj_interval[i] < 0.2 && (i == (obj_num - 1))) || (IsVehStatic[i] && IsVehStatic[i + 1] && obj_interval[i] < 0.5 && (i != (obj_num - 1))))// 0.1  0.5
			{
				que_end_pos = obj_pos[i][0];
			}
		}

	}
	//printf("id =%d,vehicle_num = %d\n",LaneID,obj_num);

	//sum = (obj_num)? sum / obj_num : 0;
	/*if(obj_num > 3 && sum < 0.1 && no_interval_num > 0)
	{
		que_pos = obj_pos[obj_num - 1][1];
	}*/
	//统计得到排队长度
	stat_num = pCfgs->uStatQuePos[LaneID][5];
	if(stat_num < 5)
	{
		pCfgs->uStatQuePos[LaneID][stat_num] = que_end_pos;
		pCfgs->uStatQuePos[LaneID][5] = pCfgs->uStatQuePos[LaneID][5] + 1;
		stat_num = stat_num + 1;
	}
	else
	{
		for(num = 1; num < 5; num++)
		{
			pCfgs->uStatQuePos[LaneID][num - 1] = pCfgs->uStatQuePos[LaneID][num];
		}
		pCfgs->uStatQuePos[LaneID][4] = que_end_pos;

	}
	//防止排队长度闪烁
	que_pos1 = sort_median(pCfgs->uStatQuePos[LaneID], stat_num);
	//printf("[%d,%d,%d,%d,%d,%d],%d\n",pCfgs->uStatQuePos[LaneID][0],pCfgs->uStatQuePos[LaneID][1],pCfgs->uStatQuePos[LaneID][2],pCfgs->uStatQuePos[LaneID][3],pCfgs->uStatQuePos[LaneID][4],pCfgs->uStatQuePos[LaneID][5],que_pos);
	if(flag == 0)//车头
	{
		if(obj_num == 0)
		{
			que_pos1 = lane_bottom;
		}
	}
	else//车尾
	{
		if(obj_num == 0)
		{
			que_pos1 = lane_top;
		}

	}
	que_end_pos = que_pos1;
	pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.uQueueHeadDis = que_start_pos;//队首
	pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.uQueueTailDis = que_end_pos;//队尾
	//计算排队区域内车辆数
	if(abs(que_end_pos - que_start_pos) > 0)//有排队
	{
		int max_pos = max(que_start_pos, que_end_pos);
		int min_pos = min(que_start_pos, que_end_pos);
		for( i = 0; i < obj_num; i++)
		{
			int dis = min(max_pos, obj_pos[i][0]) - max(min_pos, obj_pos[i][1]);
			if(dis > 0)
				QueVehicleNum++;
		}
	}
	else
	{
		QueVehicleNum = 0;
	}
	//分布情况
	pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.uVehicleDistribution = (actual_dis > 254)? 254 : actual_dis;
	//得到车辆密度
	if(sum == 0)
	{
		pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.uVehicleDensity = 0;
	}
	else
	{
		pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.uVehicleDensity = (lane_bottom - lane_top - sum) * 100 /(lane_bottom - lane_top);
	}
	pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.uVehicleDensity = (pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.uVehicleDensity < 0)? 0 : pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.uVehicleDensity;
	pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.uVehicleDensity = (pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.uVehicleDensity > 100)? 100 : pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.uVehicleDensity;
	//通道内排队数量
	pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo1.uQueueVehiSum = QueVehicleNum;//通道内排队数量
	//排队长度
	memcpy((void *)pParams->PreQueueImage, (void *)pParams->CurrQueueImage, pCfgs->team_width * pCfgs->team_height);//prePreQueueImage
}
inline void	SetImagePointValue(Int16 iCol,Int16 iRow,Uint16 Lines,Uint16 Columns,Uint16 Value,Uint8 *ImagePtr)
{
	*(ImagePtr+iRow*Columns+iCol)=Value;
}
inline Uint16 GetImagePointValue(Int16 iCol,Int16 iRow,Uint16 Lines,Uint16 Columns,Uint8 *ImagePtr)
{
	return (Uint16)*(ImagePtr+iRow*Columns+iCol);
}
//帧差
void iSubStractImage(Uint8 *puSourceImage,Uint8 *puTargetImage, Uint8 *puMaskImage, Uint32 nThreshold, Int16 laneID, Int16 width, Int16 height)
{
	Int32 iRow,iCol,nCompareResult;

	for( iRow = 0; iRow < height; iRow++ )
	{
		for( iCol = 0; iCol < width; iCol++ )
		{
			if( *(puMaskImage + iCol + width * iRow) == (laneID + 1))
			{
				nCompareResult =  *( puSourceImage + iCol + width * iRow )  -  *( puTargetImage + iCol + width * iRow ) ;
				if( abs(nCompareResult) < nThreshold )
					nCompareResult = 0;
				*( puTargetImage + iCol + width * iRow ) = (Uint8)abs(nCompareResult);
			}
		}
	}
}
//得到两点的距离
float GetLenOfTwoPoint(CPoint* Cmp_Dot1, CPoint* Cmp_Dot2)
{
	float i,j;
	j = 0;
	i=-1;
	i = (float)((Cmp_Dot1->x - Cmp_Dot2->x) * (Cmp_Dot1->x - Cmp_Dot2->x));
	i = (float)( i + (Cmp_Dot1->y - Cmp_Dot2->y) * (Cmp_Dot1->y - Cmp_Dot2->y));

	j = (float) ( sqrt( i ) );
	return (float)j;
}

//矫正图像
CPoint ptGetDot(CPoint* ptUpLeft, CPoint* ptUpRight, CPoint* ptDownRight, CPoint* ptDownLeft, Int16 nColNum, Uint32 * ptStorePlace)
{
	CPoint ptRowAndCol, ptTemp, ptOne, ptTwo;
	Uint16	MaxX,MaxY,/*cNewX,*/ uMaxLines;
	int iRow_Num;
	int iRow, iCol;//DD????D
	float fSlope;//DA???
	float temp1, temp2;
	float	fBDivder,fDivder;
	Uint32 	ImageAddr;
	Uint8 	cState;

	cState=0;
	ptOne.x = ptUpLeft->x;
	ptOne.y = ptUpLeft->y;
	ptTwo.x = ptUpRight->x;
	ptTwo.y = ptUpRight->y;

	temp1 =GetLenOfTwoPoint( &ptOne, &ptTwo );//????

	ptOne.x = ptDownLeft->x;
	ptOne.y = ptDownLeft->y;
	ptTwo.x = ptDownRight->x;
	ptTwo.y = ptDownRight->y;
	temp2 =GetLenOfTwoPoint( &ptOne, &ptTwo );//?I??

	if	(temp1 >= temp2)
	{
		cState =0x80;
	};
	if	((cState & 0x80) ==0x80)//???I?
	{
		if	((ptUpLeft->x - ptUpRight->x) ==0)
		{
			fSlope = 2.0;
		}
		else
		{
			if( abs(ptUpLeft->y - ptUpRight->y) > abs(ptUpLeft->x - ptUpRight->x) )//??1
				fSlope = 1.5;
			else
				fSlope = 0;

		};

		ptOne.x = ptDownLeft->x;
		ptOne.y = ptDownLeft->y;
		ptTwo.x = ptDownRight->x;
		ptTwo.y = ptDownRight->y;

		temp1 =GetLenOfTwoPoint( &ptOne, &ptTwo );//?I??

		// by david
		if ( (temp1 > 0) && (temp1 < (nColNum - 1)))
		{
			nColNum = (int)( temp1 + 1 );//???
		};

		// if (temp1 > 0)
		// {
		//nColNum = (int)( temp1 + 1 );//??????D??y
		// nColNum = (int)( temp1 );//??????D??y, by david 20130910
		// }
	}
	else//????I?
	{
		//I?濡楋拷?1???
		if	((ptDownRight->x - ptDownLeft->x) ==0)
		{
			fSlope = 2.0;
		}
		else
		{
			//			fSlope =(float)((float)(ptDownLeft->y - ptDownRight->y)/(float)(ptDownLeft->x - ptDownRight->x));
			if( abs(ptDownLeft->y - ptDownRight->y) > abs(ptDownLeft->x - ptDownRight->x) )//??1
				fSlope = 1.5;
			else
				fSlope = 0;
		};

		ptOne.x = ptUpLeft->x;
		ptOne.y = ptUpLeft->y;
		ptTwo.x = ptUpRight->x;
		ptTwo.y = ptUpRight->y;

		temp1 =GetLenOfTwoPoint( &ptOne, &ptTwo );//????

		// by david
		if ( (temp1 > 0) && (temp1 < (nColNum -1)))
		{
			nColNum = (int)( temp1 + 1 );//???
		};
	};

	uMaxLines = (Uint16)( MaxDotsInDetect / nColNum );
	cState =0x80;
	iRow_Num =abs(ptUpLeft->y -ptDownLeft->y);
	if	(iRow_Num > abs(ptUpRight->y - ptDownRight->y))//??
	{
		iRow_Num =abs(ptUpRight->y - ptDownRight->y);//??
		cState =cState +0x40;//2?閻曪拷?1?a3A椤ワ拷?
	};
	//	if	(abs(fSlope) >= 1)
	//	{
	//		//DA????????I?=1
	//		iRow_Num =abs(ptUpLeft->x -ptDownLeft->x);
	//		if	(iRow_Num >= abs(ptUpRight->x - ptDownRight->x))//??
	//		{
	//			iRow_Num =abs(ptUpRight->x - ptDownRight->x);//??
	//			cState =cState +0x40;//2?閻曪拷?1?a3A椤ワ拷?
	//		};
	//	}
	//	else//???1
	//	{
	//		//DA????????I?1
	//		iRow_Num =abs(ptUpLeft->y -ptDownLeft->y);
	//		if	(iRow_Num >= abs(ptUpRight->y - ptDownRight->y))//??
	//		{
	//			iRow_Num =abs(ptUpRight->y - ptDownRight->y);//??
	//			cState =cState +0x40;//2?閻曪拷?1?a3A椤ワ拷?
	//		};
	///	};
	//iRow_Num++;    //by david
	if( iRow_Num > uMaxLines )
		iRow_Num = uMaxLines;

	//??????閻烇拷?濡楋拷?o??A??閻曪拷???I???A??
	switch (cState &0xc0)
	{
	case	0x00:
		//I?濡楋拷?1??椤掞拷??2?閻曪拷?1???
		fBDivder=(((float)ptDownRight->y -(float)ptUpLeft->y)*((float)ptDownRight->x -(float)ptDownLeft->x)*((float)ptUpRight->x -(float)ptDownRight->x)\
			+ ((float)ptDownRight->y -(float)ptDownLeft->y)*((float)ptUpRight->x -(float)ptDownRight->x)*(float)ptUpLeft->x \
			- ((float)ptUpRight->y -(float)ptDownRight->y)*((float)ptDownRight->x -(float)ptDownLeft->x)*(float)ptDownRight->x);
		fDivder=(((float)ptDownRight->y -(float)ptDownLeft->y)*((float)ptUpRight->x -(float)ptDownRight->x) - ((float)ptUpRight->y -(float)ptDownRight->y)*((float)ptDownRight->x -(float)ptDownLeft->x));
		ptUpRight->x =(unsigned int)(fBDivder/fDivder);

		fBDivder=(((float)ptDownRight->x -(float)ptUpLeft->x)*((float)ptDownRight->y -ptDownLeft->y)*((float)ptUpRight->y -(float)ptDownRight->y)\
			+ ((float)ptDownRight->x -(float)ptDownLeft->x)*((float)ptUpRight->y -(float)ptDownRight->y)*(float)ptUpLeft->y \
			- ((float)ptUpRight->x -(float)ptDownRight->x)*((float)ptDownRight->y -(float)ptDownLeft->y)*(float)ptDownRight->y);
		fDivder=(((float)ptDownRight->x -(float)ptDownLeft->x)*((float)ptUpRight->y -(float)ptDownRight->y) - ((float)ptUpRight->x -(float)ptDownRight->x)*((float)ptDownRight->y -(float)ptDownLeft->y));
		ptUpRight->y =(Uint16)(fBDivder/fDivder);
		break;
	case	0x40:
		fBDivder=(((float)ptDownLeft->y -(float)ptUpRight->y)*((float)ptDownRight->x -(float)ptDownLeft->x)*((float)ptDownLeft->x -(float)ptUpLeft->x)\
			+ ((float)ptDownRight->y -(float)ptDownLeft->y)*((float)ptDownLeft->x -(float)ptUpLeft->x)*(float)ptUpRight->x \
			- ((float)ptDownLeft->y -(float)ptUpLeft->y)*((float)ptDownRight->x -(float)ptDownLeft->x)*(float)ptDownLeft->x);
		fDivder=(((float)ptDownRight->y -(float)ptDownLeft->y)*((float)ptDownLeft->x -(float)ptUpLeft->x) - ((float)ptDownLeft->y -(float)ptUpLeft->y)*((float)ptDownRight->x -(float)ptDownLeft->x));
		ptUpLeft->x =(unsigned int)(fBDivder/fDivder);

		fBDivder=(((float)ptDownLeft->x -(float)ptUpRight->x)*((float)ptDownRight->y -(float)ptDownLeft->y)*((float)ptDownLeft->y -(float)ptUpLeft->y)\
			+ ((float)ptDownRight->x -(float)ptDownLeft->x)*((float)ptDownLeft->y -(float)ptUpLeft->y)*(float)ptUpRight->y \
			- ((float)ptDownLeft->x -(float)ptUpLeft->x)*((float)ptDownRight->y -(float)ptDownLeft->y)*(float)ptDownLeft->y);
		fDivder=(((float)ptDownRight->x -(float)ptDownLeft->x)*((float)ptDownLeft->y -(float)ptUpLeft->y) - ((float)ptDownLeft->x -(float)ptUpLeft->x)*((float)ptDownRight->y -(float)ptDownLeft->y));
		ptUpLeft->y =(Uint16)(fBDivder/fDivder);

		break;
	case	0x80:
		fBDivder=(((float)ptUpRight->y -(float)ptDownLeft->y)*((float)ptUpRight->x -(float)ptUpLeft->x)*((float)ptDownRight->x -(float)ptUpRight->x)\
			+ ((float)ptUpRight->y -(float)ptUpLeft->y)*((float)ptDownRight->x -(float)ptUpRight->x)*(float)ptDownLeft->x \
			- ((float)ptDownRight->y -(float)ptUpRight->y)*((float)ptUpRight->x -(float)ptUpLeft->x)*(float)ptUpRight->x);
		fDivder=(((float)ptUpRight->y -(float)ptUpLeft->y)*((float)ptDownRight->x -(float)ptUpRight->x) - ((float)ptDownRight->y -(float)ptUpRight->y)*((float)ptUpRight->x -(float)ptUpLeft->x));
		ptDownRight->x =(Uint16)(fBDivder/fDivder);

		fBDivder=(((float)ptUpRight->x -(float)ptDownLeft->x)*((float)ptUpRight->y -(float)ptUpLeft->y)*((float)ptDownRight->y -(float)ptUpRight->y)\
			+ ((float)ptUpRight->x -(float)ptUpLeft->x)*((float)ptDownRight->y -(float)ptUpRight->y)*(float)ptDownLeft->y \
			- ((float)ptDownRight->x -(float)ptUpRight->x)*((float)ptUpRight->y -(float)ptUpLeft->y)*(float)ptUpRight->y);
		fDivder=(((float)ptUpRight->x -(float)ptUpLeft->x)*((float)ptDownRight->y -(float)ptUpRight->y) - ((float)ptDownRight->x -(float)ptUpRight->x)*((float)ptUpRight->y -(float)ptUpLeft->y));
		ptDownRight->y =(Uint16)(fBDivder/fDivder);
		break;
	default:
		fBDivder=(((float)ptUpLeft->y -(float)ptDownRight->y)*((float)ptUpRight->x -(float)ptUpLeft->x)*((float)ptDownLeft->x -(float)ptUpLeft->x)\
			+ ((float)ptUpRight->y -(float)ptUpLeft->y)*((float)ptDownLeft->x -(float)ptUpLeft->x)*(float)ptDownRight->x \
			- ((float)ptDownLeft->y -(float)ptUpLeft->y)*((float)ptUpRight->x -(float)ptUpLeft->x)*(float)ptUpLeft->x);
		fDivder=(((float)ptUpRight->y -(float)ptUpLeft->y)*((float)ptDownLeft->x -(float)ptUpLeft->x) - ((float)ptDownLeft->y -(float)ptUpLeft->y)*((float)ptUpRight->x -(float)ptUpLeft->x));
		ptDownLeft->x =(Uint16)(fBDivder/fDivder);

		fBDivder=(((float)ptUpLeft->x -(float)ptDownRight->x)*((float)ptUpRight->y -(float)ptUpLeft->y)*((float)ptDownLeft->y -(float)ptUpLeft->y)\
			+ ((float)ptUpRight->x -(float)ptUpLeft->x)*((float)ptDownLeft->y -(float)ptUpLeft->y)*(float)ptDownRight->y \
			- ((float)ptDownLeft->x -(float)ptUpLeft->x)*((float)ptUpRight->y -(float)ptUpLeft->y)*(float)ptUpLeft->y);
		fDivder=(((float)ptUpRight->x -(float)ptUpLeft->x)*((float)ptDownLeft->y -(float)ptUpLeft->y) - ((float)ptDownLeft->x -(float)ptUpLeft->x)*((float)ptUpRight->y -(float)ptUpLeft->y));
		ptDownLeft->y =(Uint16)(fBDivder/fDivder);

		break;
	};
	//MaxX=720;
	MaxX=FULL_COLS;
	MaxY=FULL_ROWS;
	////printf("row is:%d\t",iRow_Num);
	////printf("col is:%d\t",nColNum);
	////printf("upleft x:%d,y:%d\t",ptUpLeft->x,ptUpLeft->y);
	////printf("upright x:%d,y:%d\t",ptUpRight->x,ptUpRight->y);
	////printf("downleft x:%d,y:%d\t",ptDownLeft->x,ptDownLeft->y);
	////printf("downright x:%d,y:%d\n",ptDownRight->x,ptDownRight->y);
	for( iRow = 0; iRow < iRow_Num; iRow++)
	{
		for( iCol = 0; iCol < nColNum ; iCol++ )
		{
			/*ptTemp.x=(Uint16)( (float)ptUpLeft->x + ((float)ptDownLeft->x - (float)ptUpLeft->x)*iRow/(iRow_Num -1)\
			+((float)ptUpRight->x - (float)ptUpLeft->x)*iCol/(nColNum -1)\
			+((float)ptUpLeft->x + (float)ptDownRight->x - (float)ptUpRight->x - (float)ptDownLeft->x)*iCol/((iRow_Num -1) *(nColNum -1)) );

			ptTemp.y=(Uint16)( (float)ptUpLeft->y + ((float)ptDownLeft->y - (float)ptUpLeft->y)*iRow/(iRow_Num -1)\
			+((float)ptUpRight->y -(float)ptUpLeft->y)*iCol/(nColNum -1)\
			+((float)ptUpLeft->y + (float)ptDownRight->y - (float)ptUpRight->y - (float)ptDownLeft->y)*iCol/((iRow_Num -1) *(nColNum -1)) );*/
			temp1=(float)ptUpRight->x + ((float)ptDownRight->x-(float)ptUpRight->x)*iRow/(iRow_Num -1);
			temp2=(float)ptUpLeft->x + ((float)ptDownLeft->x - (float)ptUpLeft->x)*iRow/(iRow_Num -1);
			ptTemp.x=(Uint16)( temp2+(temp1-temp2)*iCol/(nColNum-1)+((float)ptUpLeft->x + (float)ptDownRight->x - (float)ptUpRight->x - (float)ptDownLeft->x)*iCol/((iRow_Num -1) *(nColNum -1)));

			ptTemp.y=(Uint16)( (float)ptUpLeft->y + ((float)ptDownLeft->y - (float)ptUpLeft->y)*iRow/(iRow_Num -1)\
				+((float)ptUpRight->y -(float)ptUpLeft->y)*iCol/(nColNum -1)\
				+((float)ptUpLeft->y + (float)ptDownRight->y - (float)ptUpRight->y - (float)ptDownLeft->y)*iCol/((iRow_Num -1) *(nColNum -1)) );
			//ptTemp.y=ptUpLeft->y+iRow;
			//if (iRow==0)
			//{
			//	if(iCol ==0)
			//	{
			//		ptUpLeft->x=ptTemp.x;
			//		ptUpLeft->y=ptTemp.y;
			//	}
			//	if (iCol ==(nColNum-1))
			//	{
			//		//temp1=ptTemp.x;
			//		ptUpRight->x=ptTemp.x;
			//		ptUpRight->y=ptTemp.y;
			//	}
			//}
			//if(iRow==(iRow_Num-1))
			//{
			//	if(iCol ==0)
			//	{
			//		ptDownLeft->x=ptTemp.x;
			//		ptDownLeft->y=ptTemp.y;
			//	}
			//	if (iCol ==(nColNum-1))
			//	{
			//		//temp2=ptTemp.x;
			//		ptDownRight->x=ptTemp.x;
			//		ptDownRight->y=ptTemp.y;
			//	}
			//}

			if(ptTemp.x>MaxX||ptTemp.y>MaxY)
			{
				break;
			}
			ImageAddr=((Uint32)ptTemp.y*(Uint32)MaxX)+((Uint32)ptTemp.x);

			*(ptStorePlace+iRow*nColNum+iCol) = ImageAddr;

		}
	}


	ptRowAndCol.x = (Uint16)nColNum;//
	ptRowAndCol.y = iRow_Num;//


	return ptRowAndCol;
}

//矩阵相乘1203
void matrix_mult( float *result, float *left, float *right, int m, int n, int z )
{
	int i, j, k;
	float sum;
	for( i = 0; i < m; i++ )
		for( j =0; j < z; j++ )
		{
			sum = 0.0;
			for( k = 0; k < n; k++ )
				sum = sum + ( left[ i*n+k ] * right[ k*z+j ] );
			result[i*z+j] = sum;
		}
}
//矩阵转置
void matrix_transport( float *result, float *mat, int m, int n )
{
	int i, j;
	for( i = 0; i < n; i++ )
		for( j =0; j < m; j++ )
			result[i*m+j] = mat[j*n+i];
}
//3*3矩阵求逆
void matrix_inverse(float *R, float *Ri)
{
	float den=(R[0]*R[4]*R[8] + R[1]*R[5]*R[6] + R[2]*R[3]*R[7])-
		(R[2]*R[4]*R[6] + R[1]*R[3]*R[8] + R[0]*R[5]*R[7]);
	int   i;
	if((den<-0.00000000001)||(den>0.00000000001))
	{
		Ri[0]=(R[4]*R[8]-R[7]*R[5])/den;
		Ri[1]=(R[2]*R[7]-R[1]*R[8])/den;
		Ri[2]=(R[1]*R[5]-R[2]*R[4])/den;
		Ri[3]=(R[5]*R[6]-R[3]*R[8])/den;
		Ri[4]=(R[0]*R[8]-R[2]*R[6])/den;
		Ri[5]=(R[2]*R[3]-R[0]*R[5])/den;
		Ri[6]=(R[3]*R[7]-R[4]*R[6])/den;
		Ri[7]=(R[1]*R[6]-R[0]*R[7])/den;
		Ri[8]=(R[0]*R[4]-R[1]*R[3])/den;
	}
	else 
	{
		for(i=0; i<9; i++) 
		{
			Ri[i]=0.0;

		}
	}
}

//??
bool jacobi(float *a, float *eigen_val, float *eigen_vec,int n)
{
	int k = 0,i,j;
	const float e = 0.00001;		//?
	const int l = 10000;			//
	int p, q;
	float max_value =0;
	float cos_2a, sin_2a, cos_a, sin_a;
	float t, z;
	float a_pp;
	float a_qq;
	float a_pq;
	float a_pi;
	float a_qi;
	float r_ip;
	float r_iq;
	for (i = 0; i < n; i++) 
		eigen_vec[i * n + i] = 1;
	while (1)
	{
		max_value = 0;
		for (i = 0; i < n; i++)
			for (j = i + 1; j < n; j++)
				if (fabs(a[i * n + j]) > max_value)
				{
					max_value = fabs(a[i * n + j]);
					p = i;
					q = j;
				}
				if (max_value < e || k > l) break;


				if (fabs(a[p * n + p] - a[q * n + q]) == 0)
				{
					sin_2a = 1;
					cos_2a = 0;
					cos_a = 1 / sqrt(2.0);
					sin_a = 1 / sqrt(2.0);
				}
				else 
				{	
					t = 2 * a[p * n + q] / (a[p * n + p] - a[q * n + q]);
					z = (a[p * n + p] - a[q * n + q]) / (2 * a[p * n + q]);

					if (fabs(t) < 1)
					{
						cos_2a = 1 / sqrt(1 + t * t);
						sin_2a = t / sqrt(1 + t * t);
					}
					else 
					{
						cos_2a = fabs(z) / sqrt(1 + z * z);
						sin_2a = (z > 0 ? 1 : (z < 0 ? -1 :0)) / sqrt(1 + z * z);
					}
					cos_a = sqrt((1 + cos_2a) / 2);
					sin_a = sin_2a / (2 * cos_a);
				}
				a_pp =a[p * n + p];
				a_qq = a[q * n + q];
				a_pq = a[p * n + q];
				a[p * n + p] = a_pp * cos_a * cos_a + a_qq * sin_a * sin_a + 2 * a_pq * cos_a * sin_a;
				a[q * n + q] = a_pp * sin_a * sin_a + a_qq * cos_a * cos_a - 2 * a_pq * cos_a * sin_a;

				for (i = 0; i < n; i++)
					if (i != p && i != q)
					{
						a_pi = a[p * n + i];
						a_qi = a[q * n + i];

						a[p * n + i] = a[i * n + p] = a_pi * cos_a + a_qi * sin_a;
						a[q * n + i] = a[i * n + q] = - a_pi * sin_a + a_qi * cos_a;
					}

					a[p * n + q] = a[q * n + p] = (a_qq - a_pp) * sin_2a / 2 + a_pq * cos_2a;
					//?
					for (i = 0; i < n; i++)
					{
						r_ip = eigen_vec[i * n + p];
						r_iq = eigen_vec[i * n + q];
						eigen_vec[i * n + p] = r_ip * cos_a + r_iq * sin_a;
						eigen_vec[i * n + q] = - r_ip * sin_a + r_iq * cos_a;
					}
					k++;
	}

	for (i = 0; i < n; i++) 
		eigen_val[i] = a[i * n + i];
	return TRUE;
}
//svd?
void svd( float *a, int m, int n,  float *d, float v[] )
{
	int i, j, k;
	float aT[2*CALIBRATION_POINT_NUM*9]={0};
	float aT_a[9*9] ={0};
	float tmp;
	float t[9] = {0};
	matrix_transport( aT, a, m, n );
	matrix_mult( aT_a, aT, a, n, m, n );
	jacobi(aT_a, d, v,n);
	//???

	for( i = 0; i < n-1; i++ )
	{
		tmp = d[ i ];
		for( k = 0; k < n; k++ )
			t[ k ] = v[ k * n + i ];
		for( j = i+1; j < n ; j++ )
		{
			if( d[ j ] > tmp )
			{
				d[ i ] = d[ j ];
				d[ j ] = tmp;
				tmp = d[ i ];
				for( k = 0; k < n; k++ )
				{
					v[ k * n + i ] = v[ k * n + j ];
					v[ k * n + j ] = t[ k ];
				}				
			}
		}
	}

}

//lhx,20150608????
/*static void camera_calibration(float actual_point[][2],float img_point[][2],float mapping_matrix[],int calibration_num,ALGCFGS *pCfgs)
{
	int sub_calibration_num=4;
	int subsection_num=(calibration_num-sub_calibration_num)/2+1;
	int num=0;
	float tx=0,ty=0,tu=0,tv=0;
	float s1=0,s2=0;
	float T1[9]={0};
	float T2[9]={0};
	int i,j;

	float normalization_actual_x[4];
	float normalization_actual_y[4];
	float normalization_img_x[4];
	float normalization_img_y[4];
	float A[2*4*9]={0};
	float v[9*9]={0};
	float d[9]={0};
	float L[9]={0};
	float T1_inv[9]={0};
	float temp[9]={0};
	int overlap_row1,overlap_row2,flag=0;
	//??????
	for(num=0;num<subsection_num;num++)
	{
		//???
		tx=ty=tu=tv=0;
		s1=s2=0;		
		for(i=0;i<sub_calibration_num;i++)
		{
			j=i+num*2;
			tx=tx+actual_point[j][0];
			ty=ty+actual_point[j][1];
			tu=tu+img_point[j][0];
			tv=tv+img_point[j][1];
		}
		tx=tx/sub_calibration_num;
		ty=ty/sub_calibration_num;
		tu=tu/sub_calibration_num;
		tv=tv/sub_calibration_num;
		for(i=0;i<sub_calibration_num;i++)
		{
			j=i+num*2;
			s1=s1+sqrt((img_point[j][0]-tu)*(img_point[j][0]-tu)+(img_point[j][1]-tv)*(img_point[j][1]-tv));
			s2=s2+sqrt((actual_point[j][0]-tx)*(actual_point[j][0]-tx)+(actual_point[j][1]-ty)*(actual_point[j][1]-ty));
		}
		s1=(fabs(s1)<1e-6)?0:sqrt(2.0)/s1;
		s2=(fabs(s2)<1e-6)?0:sqrt(2.0)/s2;
		T1[1]=T1[3]=T1[6]=T1[7]=0;
		T1[0]=T1[4]=s1;
		T1[2]=-s1*tu;
		T1[5]=-s1*tv;
		T1[8]=T2[8]=1;
		T2[1]=T2[3]=T2[6]=T2[7]=0;
		T2[0]=T2[4]=s2;
		T2[2]=-s2*tx;
		T2[5]=-s1*ty;
		//?????	
		for(i=0;i<sub_calibration_num;i++)
		{
			j=i+num*2;
			normalization_img_x[i]=T1[0]*img_point[j][0]+T1[1]*img_point[j][1]+T1[2];
			normalization_img_y[i]=T1[3]*img_point[j][0]+T1[4]*img_point[j][1]+T1[5];
			normalization_actual_x[i]=T2[0]*actual_point[j][0]+T2[1]*actual_point[j][1]+T2[2];
			normalization_actual_y[i]=T2[3]*actual_point[j][0]+T2[4]*actual_point[j][1]+T2[5];

		}
		//????,?????

		for(i=0;i<2*4*9;i++)
		{
			A[i]=0.0;
		}
		for(i=0;i<sub_calibration_num;i++)
		{
			A[(2*i)*9+0]=normalization_actual_x[i];
			A[(2*i)*9+1]=normalization_actual_y[i];
			A[(2*i)*9+2]=1;
			A[(2*i)*9+6]=-1*normalization_img_x[i]*normalization_actual_x[i];
			A[(2*i)*9+7]=-1*normalization_img_x[i]*normalization_actual_y[i];
			A[(2*i)*9+8]=-1*normalization_img_x[i];
			A[(2*i+1)*9+3]=normalization_actual_x[i];
			A[(2*i+1)*9+4]=normalization_actual_y[i];
			A[(2*i+1)*9+5]=1;
			A[(2*i+1)*9+6]=-1*normalization_img_y[i]*normalization_actual_x[i];
			A[(2*i+1)*9+7]=-1*normalization_img_y[i]*normalization_actual_y[i];
			A[(2*i+1)*9+8]=-1*normalization_img_y[i];

		}
		for(i=0;i<9*9;i++)
		{
			v[i]=0.0;
		}
		for(i=0;i<9;i++)
		{
			d[i]=0.0;
		}
		//svd??
		svd( A, 2*sub_calibration_num, 9, d, v );
		//v???????????????
		//	float L[9]={0};
		for( i = 0; i < 9; i++ )
			L[i] = v[9*i+8];
		//???????????inv(T1)*L*T2;
		//		float T1_inv[9]={0};
		for(i=0;i<9;i++)
		{
			T1_inv[i]=0.0;
		}
		matrix_inverse(T1, T1_inv);

		for(i=0;i<9;i++)
		{
			temp[i]=0.0;

		}


		matrix_mult( temp, T1_inv, L, 3,3,3 );
		matrix_mult(mapping_matrix,temp,T2,3,3,3);
		flag=0;
		if(num==0)
		{
			flag=1;
			overlap_row1=0;
			overlap_row2=0;
		}
		else
		{
			if(num==subsection_num-1)
				flag=2;
			overlap_row1=(img_point[num*2][1]+img_point[num*2+1][1])/2;
			overlap_row2=(img_point[num*2+sub_calibration_num-4][1]+img_point[num*2+sub_calibration_num-3][1])/2;
		}
		img_to_actual(mapping_matrix,(img_point[num*2][1]+img_point[num*2+1][1])/2,(img_point[num*2+sub_calibration_num-2][1]+img_point[num*2+sub_calibration_num-1][1])/2,overlap_row1,overlap_row2,flag,pCfgs);
	}

}*/
//对点进行排序
void sort_point(int array[][2],int length)
{
	int i=0,j=0;
	int temp[2];
	for(i=0;i<length-1;i++)
	{
		for(j=i+1;j<length;j++)
		{
			if(array[i][1]>array[j][1])
			{
				temp[0]=array[i][0];
				temp[1]=array[i][1];
				array[i][0]=array[j][0];
				array[i][1]=array[j][1];
				array[j][0]=temp[0];
				array[j][1]=temp[1];
			}
		}
	}
	if(array[0][0]>array[1][0])
	{
		temp[0]=array[0][0];
		temp[1]=array[0][1];
		array[0][0]=array[1][0];
		array[0][1]=array[1][1];
		array[1][0]=temp[0];
		array[1][1]=temp[1];
	}
	if(array[2][0]>array[3][0])
	{
		temp[0]=array[2][0];
		temp[1]=array[2][1];
		array[2][0]=array[3][0];
		array[2][1]=array[3][1];
		array[3][0]=temp[0];
		array[3][1]=temp[1];
	}

}
//对相机图像进行标定
static void camera_calibration(int base_line[][2], float base_length, int calibration_point[][2], float near_point_length, int laneNum, ALGCFGS *pCfgs, int imgW, int imgH)
{
	int i = 0, j = 0;
	//标定区域直线0——1——3——2——0
	float k01 = 0, b01 = 0;
	float k13 = 0, b13 = 0;
	float k32 = 0, b32 = 0;
	float k20 = 0, b20 = 0;
	//标定区域两条线的交点0-2 1-3
	float x0 = 0, y0 = 0;
	float dis = 1;
	//检测车道中心线斜率
	float k_road = 0, b_road = 0;
	//float k_base = 0, b_base = 0;
	float pt_x = 0, pt_y = 0, pt1_x = 0, pt1_y = 0, pt2_x = 0, pt2_y = 0, pt3_x = 0, pt3_y = 0;

	int temp, temp0 = 0;
	int start_point = 0, end_point = 0;
	float calibration_base_line[MAX_IMAGE_HEIGHT] = { 0 };
	float dis_pixel[MAX_IMAGE_HEIGHT] = { 0 };
	float temp1 = 0, temp2 = 0;
	//对标定区域点进行排序
	sort_point(calibration_point, 4);
	//求标定区域直线斜率和截距
	if(calibration_point[0][0] != calibration_point[1][0])
	{
		k01 = 1.0 * (calibration_point[0][1] - calibration_point[1][1]) / (calibration_point[0][0] - calibration_point[1][0]);
		b01 = calibration_point[0][1] - k01 * calibration_point[0][0];
	}
	else
	{
		k01 = 1e+6;
	}
	if(calibration_point[1][0] != calibration_point[3][0])
	{
		k13 = 1.0 * (calibration_point[1][1] - calibration_point[3][1]) / (calibration_point[1][0] - calibration_point[3][0]);
		b13 = calibration_point[1][1] - k13 * calibration_point[1][0];
	}
	else
	{
		k13 = 1e+6;
	}
	if(calibration_point[3][0] != calibration_point[2][0])
	{
		k32 = 1.0 * (calibration_point[3][1] - calibration_point[2][1]) / (calibration_point[3][0] - calibration_point[2][0]);
		b32 = calibration_point[3][1] - k32 * calibration_point[3][0];
	}
	else
	{
		k32 = 1e+6;
	}
	if(calibration_point[2][0] != calibration_point[0][0])
	{
		k20 = 1.0 * (calibration_point[2][1] - calibration_point[0][1]) / (calibration_point[2][0] - calibration_point[0][0]);
		b20 = calibration_point[2][1] - k20 * calibration_point[2][0];
	}
	else
	{
		k20 = 1e+6;
	}
	if(base_line[0][1] > base_line[1][1])
	{
		start_point = base_line[0][1];
		end_point = base_line[1][1];
	}

	else
	{
		start_point = base_line[1][1];
		end_point = base_line[0][1];
	}
	//两条车道线是垂直线
	if(k20 > 1e+5 && k13 > 1e+5)
	{
		dis = base_length / (start_point - end_point + 1);
		calibration_base_line[imgH - 1] = near_point_length;
		for(i = imgH - 2; i >= 0; i--)
		{
			calibration_base_line[i] = calibration_base_line[i + 1] + dis;
			dis_pixel[i] = dis;
		}
	}
	else
	{
		if(k20 > 1e+5)
		{
			x0 = calibration_point[0][0];
			y0 = k13 * x0 + b13;
		}
		else if(k13 > 1e+5)
		{
			x0 = calibration_point[1][0];
			y0 = k20 * x0 + b20;
		}
		else
		{
			x0 = (b13 - b20) / (k20 - k13);
			y0 = k13 * x0 + b13;
		}
		//对基准线上进行标定
		temp1 = 0, temp2 = 0;
		temp1 = (float)(imgH - 1 - end_point) / (float)(end_point - y0);
		temp2 = (float)(imgH - 1 - start_point) / (float)(start_point - y0);
		dis = (float)((base_line[1][0] - base_line[0][0]) * (base_line[1][0] - base_line[0][0]));
		dis = (float)(dis + (base_line[1][1] - base_line[0][1]) * (base_line[1][1] - base_line[0][1]));
		dis = sqrt(dis);
		base_length = base_length * (float)(start_point - end_point + 1) / dis;
		dis = base_length / (temp1 - temp2);
		calibration_base_line[imgH - 1] = near_point_length;
		for(i = imgH - 2; i >= 0; i--)
		{
			calibration_base_line[i] = near_point_length + dis * (float)(imgH - 1 - i) / (float)(i - y0);
			dis_pixel[i] = calibration_base_line[i] - calibration_base_line[i + 1];
		}
	}
	//每个车道进行标定
	/*if(base_line[1][0] != base_line[0][0])
	{
	k_base = 1.0 * (base_line[1][1] - base_line[0][1]) / (base_line[1][0] - base_line[0][0]);
	b_base = base_line[1][1] - k_base * base_line[1][0];
	pt_x = (b01 - b_base) / (k_base - k01);
	pt_y = k01 * pt_x + b01;

	}
	else
	{
	pt_x = base_line[0][0];
	pt_y = k01 * pt_x + b01;
	}*/
	pt_x = x0;
	pt_y = k01 * pt_x + b01;
	for(i=0;i<laneNum;i++)
	{

		pt1_x = (pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[0].x + pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[1].x) / 2;
		pt1_y = (pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[0].y + pCfgs->DownSpeedCfg.SpeedEachLane[i].MiddleCoil[1].y) / 2;
		pt2_x = (pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[2].x + pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[3].x) / 2;
		pt2_y = (pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[2].y + pCfgs->DownSpeedCfg.SpeedEachLane[i].RearCoil[3].y) / 2;
		if(pt1_x != pt2_x)
		{
			k_road = 1.0 * (pt1_y - pt2_y) / (pt1_x - pt2_x);
			b_road = pt1_y - k_road * pt1_x;
			//求交点
			pt3_x = (b01-b_road) / (k_road - k01);
			pt3_y = k01 * pt3_x + b01;
		}
		else
		{
			//求交点
			pt3_x = pt1_x;
			pt3_y = k01 * pt3_x + b01;
		}
		temp = (int)(pt3_y + 0.5);
		temp = MAX(0, temp);
		temp = MIN(temp, imgH - 1);
		temp0 = MAX(0, (int)(pt_y + 0.5));
		temp0 = MIN(temp0, imgH - 1);
		pCfgs->actual_distance[i][temp] = calibration_base_line[temp0];
		for(j = temp + 1; j < imgH; j++)
		{
			pCfgs->actual_distance[i][j] = pCfgs->actual_distance[i][j - 1] - dis_pixel[j - 1];
		}
		for(j = temp - 1; j >= 0; j--)
		{
			pCfgs->actual_distance[i][j] = pCfgs->actual_distance[i][j + 1] + dis_pixel[j];
		}
	}
	for(j = 0; j < imgH; j++)
	{
		pCfgs->actual_degree_length[j] = pCfgs->actual_distance[laneNum - 1][j];
	}
	//对整个图像区域进行标定
	for(i = 0; i < imgW; i++)
	{
		if(k01 > 1e+5)//垂直线
		{

		}
		else
		{
			pt3_y = k01 * i + b01;
		}
		temp = (int)(pt3_y + 0.5);
		temp = MAX(0, temp);
		temp = MIN(temp, imgH - 1);
		temp0 = MAX(0, (int)(pt_y + 0.5));
		temp0 = MIN(temp0, imgH - 1);
		pCfgs->image_actual[temp][i][1] = calibration_base_line[temp0];
		for(j = temp + 1; j < imgH; j++)
		{
			pCfgs->image_actual[j][i][1] = pCfgs->image_actual[j - 1][i][1] - dis_pixel[j - 1];
		}
		for(j = temp - 1; j >= 0; j--)
		{
			pCfgs->image_actual[j][i][1] = pCfgs->image_actual[j + 1][i][1] + dis_pixel[j];
		}
	}
}
//对相机图像进行标定
//camera_height 摄像机安装的实际高度
//near_point_length 图像最近端到相机的水平距离
//far_point_length 图像最远端到相机的水平距离
static void camera_calibration(float camera_height, float near_point_length, float far_point_length, ALGCFGS *pCfgs, int imgW, int imgH)
{
	int i = 0;
	float angle1,angle2, angle3;
	float m;//相机到图像平面的垂直距离
	angle1 = atan(near_point_length / camera_height);//最近端与相机的夹角
	angle2 = atan(far_point_length / camera_height);//最远端与相机的夹角
	m = imgH / (2 * tan((angle2 - angle1)/2));
	for(i = 0; i < imgH; i++)
	{
		angle3 = angle1 + (angle2 - angle1) / 2 + atan(imgH / 2 - i) / m;
		pCfgs->actual_distance[0][i] = tan(angle3) * camera_height;
	}
}
//从图像坐标向实际坐标进行映射lhx,20150608
static void img_to_actual(float mapping_matrix[],int start_row,int end_row,int overlap_row1,int overlap_row2,int flag,ALGCFGS *pCfgs, int imgW, int imgH)
{
	int row,col;
	float a1=0,a2=0;
	float b1=0,b2=0;
	float c1=0,c2=0;
	int temp;
	float temp1=0;
	float temp2=0;
	if(flag==1)
	{
		if(start_row>=end_row)
			start_row=imgH-1;
		else
			start_row=0;
	}
	if(flag==2)
	{
		if(start_row>=end_row)
			end_row=0;
		else	
			end_row=imgH-1;
	}
	if(start_row>end_row)
	{
		temp=start_row;
		start_row=end_row;
		end_row=temp;
	}
	if(overlap_row1>overlap_row2)
	{
		temp=overlap_row1;
		overlap_row1=overlap_row2;
		overlap_row2=temp;
	}
	for(row=start_row;row<=end_row;row++)
	{
		for(col=0;col<imgW;col++)
		{
			a1=mapping_matrix[0]-mapping_matrix[6]*col;
			a2=mapping_matrix[3]-mapping_matrix[6]*row;
			b1=mapping_matrix[1]-mapping_matrix[7]*col;
			b2=mapping_matrix[4]-mapping_matrix[7]*row;
			c1=mapping_matrix[2]-mapping_matrix[8]*col;
			c2=mapping_matrix[5]-mapping_matrix[8]*row;
			temp1=0;
			temp2=0;
			if(abs(a1*b2-b1*a2)>0.00000000001)
			{
				temp1=(a2*c1-a1*c2)/(a1*b2-b1*a2);
				temp2=(b1*c2-b2*c1)/(a1*b2-b1*a2);
				if(row>=overlap_row1&&row<=overlap_row2&&flag!=1)
				{
					pCfgs->image_actual[row][col][0]+=temp1;
					pCfgs->image_actual[row][col][1]+=temp2;
					pCfgs->image_actual[row][col][0]=pCfgs->image_actual[row][col][0]/2;
					pCfgs->image_actual[row][col][1]=pCfgs->image_actual[row][col][1]/2;
				}
				else
				{
					pCfgs->image_actual[row][col][0]=temp1;
					pCfgs->image_actual[row][col][1]=temp2;
				}
			}
			else
			{
				pCfgs->image_actual[row][col][0]=0;
				pCfgs->image_actual[row][col][1]=0;
			}

		}
	}
}
//得到实际点的距离值
static void get_actual_point(float actual_point[2],int row,int col,int limit_line,ALGCFGS *pCfgs)
{
	row=(row<limit_line)? limit_line:row;
	actual_point[0]=pCfgs->image_actual[row][col][0];
	actual_point[1]=pCfgs->image_actual[row][col][1];

}
//计算两点的实际距离
float distance_two(float actual_point1[2],float actual_point2[2])
{
	float dis=0;
	dis=(actual_point1[0]-actual_point2[0])*(actual_point1[0]-actual_point2[0])+(actual_point1[1]-actual_point2[1])*(actual_point1[1]-actual_point2[1]);
	dis=sqrt(dis);
	return dis;

}
///////////////////////////////////////////////
float fuzzy(unsigned char* puNewImage,int nWidth,int nHight)//计算视频对比度
{
	float degree=0.0;
	int i,j;
	unsigned char x1,x2,x3;
	float temp=0.0;
	int count=0;
	for(i=100;i<nHight - 100;i+=4)  
	{  
		for(j=0;j<nWidth;j+=4)  
		{  
			x1=*(puNewImage+i*nWidth+j);
			x2=*(puNewImage+(i+1)*nWidth+j);
			x3=*(puNewImage+i*nWidth+j+1);
			degree=(x2-x1)*(x2-x1)+(x3-x1)*(x3-x1);
			temp+=sqrt(degree);
			temp+=abs(x2-x1)+abs(x3-x1);
			count++;
		}  
	}  
	degree=temp/count;
	return degree;
}

//计算视频图像颜色异常
bool Color_deviate(unsigned char* uImage,unsigned char* vImage,int width,int height)
{	
	float ave_a=0,ave_b=0,std_a=0,std_b=0;
	int x=0,y=0;
	float color_deviate=0;
	int pixelnum=0;
	int temp_a,temp_b;
	//pixelnum=width*height;
	for (y = 100; y < height - 100; y += 4)
	{
		for (x = 0; x < width; x += 4)
		{
			ave_a += *(uImage + y * width + x) - 128;
			ave_b += *(vImage + y * width + x) - 128;
			pixelnum++;
		}
	} 
	ave_a /= pixelnum;
	ave_b /= pixelnum;


	for (y = 100; y < height - 100; y += 4)
	{
		for (x = 0; x < width; x += 4)
		{
			temp_a = *(uImage + y * width + x) - 128;
			std_a += (temp_a - ave_a) * (temp_a - ave_a);
			temp_b = *(vImage + y * width + x) - 128;
			std_b += (temp_b - ave_b) * (temp_b - ave_b);
		}
	}
	std_a /= pixelnum;
	std_b /= pixelnum;
	color_deviate=sqrt(ave_a*ave_a+ave_b*ave_b)/sqrt(std_a+std_b);
	//printf("\ncolor deviate is:%f\n,",color_deviate*10);

	if (color_deviate>=5||color_deviate<0.05)
	{
		return TRUE;

	} 
	else
	{
		return FALSE; 
	}

}
//统计能见度
bool visible_judge(Uint16 *a,int visib_length,int threshold)
{
	int i=0,num=0;
	for (i=0;i<visib_length;i++)
	{
		if (a[i]<threshold)
		{
			num++;
		}
		else
		{
			break;
		}

	}
	//当能见度大于数组长度一半以上，才认为是能见度高
	if (num>(visib_length/2))
	{
		return TRUE;
	} 
	else
	{
		return FALSE;
	}
}
