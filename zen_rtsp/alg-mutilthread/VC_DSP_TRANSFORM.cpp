//#include "StdAfx.h"
#include <stdio.h>
#include <stdlib.h> /// div_t 碌脛脡霉脙梅
#include <string.h>
#include "m_arith.h"
#include "DSPARMProto.h"
#include <math.h>
#include<iostream>
#include<fstream>
#include <sys/time.h>
#include "../common.h"
//
//SPEEDCFGSEG    pDetectCfgSeg;
//CFGINFOHEADER  pCfgHeader;
//RESULTMSG outbuf;




using namespace std;

#define POINTSIZE 16
CIVD_SDK_API bool transform_init_DSP_VC(bool iniflag, Uint16 lanecount,
		LANEINISTRUCT LaneIn,RESULTMSG *p_outbuf,m_args *p_arg)
{
	SPEEDCFGSEG    *p_pDetectCfgSeg=&p_arg->pDetectCfgSeg;
	CFGINFOHEADER  *p_pCfgHeader=&p_arg->pCfgHeader;

	CPoint *ptimage=p_arg->ptimage;
	CPoint *m_ptend=p_arg->m_ptEnd;

	CPoint	ptactual[8];
	// RESULTMSG *p_outbuf=( RESULTMSG *)p_arg->p_outbuf;
	alg_mem_malloc(p_arg);
	ALGPARAMS *pParams=p_arg->pParams;
	ALGCFGS *pCfgs=p_arg->pCfgs;
	Uint16 ChNum=0;
	bool   flag_ini=false;
	p_pCfgHeader->uDetectPosition=2;//txl,20150709,2->1
	p_pCfgHeader->uDetectFuncs[0]=0x00000001;
	p_pCfgHeader->uDetectFuncs[1]=0;
	/*************************************************************************************************************/
	p_pDetectCfgSeg->uNum=1;
	p_pDetectCfgSeg->uType=1;
	p_pDetectCfgSeg->uSegData[0].uLaneTotalNum=lanecount;//锟斤拷锟矫筹拷锟斤拷锟斤拷锟斤拷
	for(int i=0;i<8;i++)
	{
		p_pDetectCfgSeg->uSegData[0].ptactual[i].x=ptactual[i].x;
		p_pDetectCfgSeg->uSegData[0].ptactual[i].y=ptactual[i].y;
		p_pDetectCfgSeg->uSegData[0].ptimage[i].x=ptimage[i].x;
		p_pDetectCfgSeg->uSegData[0].ptimage[i].y=ptimage[i].y;

	}
	//memset((void *)&pCfgs,0,sizeof(pCfgs));
	for(unsigned int i=0;i<p_pDetectCfgSeg->uSegData[0].uLaneTotalNum;i++)//每锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
	{
		if(iniflag)
		{
			//流量线圈
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[0].x = m_ptend[8+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[0].y = m_ptend[8+i*POINTSIZE].y;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[1].x = m_ptend[9+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[1].y = m_ptend[9+i*POINTSIZE].y;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[2].x = m_ptend[11+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[2].y = m_ptend[11+i*POINTSIZE].y;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[3].x = m_ptend[10+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[3].y = m_ptend[10+i*POINTSIZE].y;

			//占有线圈
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].MiddleCoil[0].x = m_ptend[12+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].MiddleCoil[0].y = m_ptend[12+i*POINTSIZE].y;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].MiddleCoil[1].x = m_ptend[13+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].MiddleCoil[1].y = m_ptend[13+i*POINTSIZE].y;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].MiddleCoil[2].x = m_ptend[15+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].MiddleCoil[2].y = m_ptend[15+i*POINTSIZE].y;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].MiddleCoil[3].x = m_ptend[14+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].MiddleCoil[3].y = m_ptend[14+i*POINTSIZE].y;

			//占位线圈
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].FrontCoil[0].x = m_ptend[4+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].FrontCoil[0].y = m_ptend[4+i*POINTSIZE].y;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].FrontCoil[1].x = m_ptend[5+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].FrontCoil[1].y = m_ptend[5+i*POINTSIZE].y;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].FrontCoil[2].x = m_ptend[7+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].FrontCoil[2].y = m_ptend[7+i*POINTSIZE].y;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].FrontCoil[3].x = m_ptend[6+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].FrontCoil[3].y = m_ptend[6+i*POINTSIZE].y;

 			//车道区域
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].LaneRegion[0].x = m_ptend[0+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].LaneRegion[0].y = m_ptend[0+i*POINTSIZE].y;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].LaneRegion[1].x = m_ptend[1+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].LaneRegion[1].y = m_ptend[1+i*POINTSIZE].y;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].LaneRegion[2].x = m_ptend[2+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].LaneRegion[2].y = m_ptend[2+i*POINTSIZE].y;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].LaneRegion[3].x = m_ptend[3+i*POINTSIZE].x;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].LaneRegion[3].y = m_ptend[3+i*POINTSIZE].y;
		}

		else{
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[0].x =410;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[0].y =200;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[1].x = 580;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[1].y = 200;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[2].x = 580;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[2].y = 390;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[3].x = 410;
			p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].RearCoil[3].y = 390;
		}
			//printf("ptCornerQ[0] is:%d,%d\n",m_ptend[0].x,m_ptend[0].y);
			//printf("ptCornerQ[1]is:%d,%d\n",m_ptend[1].x,m_ptend[1].y);
			//printf("ptCornerQA[0]is:%d,%d\n",m_ptend[6].x,m_ptend[6].y);
			//printf("ptCornerQA[1]is:%d,%d\n",m_ptend[7].x,m_ptend[7].y);
			//printf("RearCoil[0] is:%d,%d\n",m_ptend[4].x,m_ptend[4].y);
			//printf("RearCoil[1]is:%d,%d\n",m_ptend[5].x,m_ptend[5].y);
			//printf("RearCoil[2]is:%d,%d\n",m_ptend[2].x,m_ptend[2].y);
			//printf("RearCoil[3]is:%d,%d\n",m_ptend[3].x,m_ptend[3].y);
		p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].uDetectDerection=2;////1锟角筹拷尾 2锟角筹拷头

		p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].ptFrontLine=0;
		p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].ptBackLine=0;
		//p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].uTransFactor=2.0;//锟斤拷圈实锟绞碉拷锟斤拷锟�?锟斤拷
		//p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].uGraySubThreshold=40;//200
		//p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].uSpeedCounterChangedThreshold=20;
		//p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].uSpeedCounterChangedThreshold1=20;
		//p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].uSpeedCounterChangedThreshold2=20;

		p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].uTransFactor=LaneIn.uTransFactor;//锟斤拷圈实锟绞碉拷锟斤拷锟�?锟斤拷
		p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].uGraySubThreshold=LaneIn.uGraySubThreshold;//200
		p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].uSpeedCounterChangedThreshold=LaneIn.uSpeedCounterChangedThreshold;
		p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].uSpeedCounterChangedThreshold1=LaneIn.uSpeedCounterChangedThreshold1;
		p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].uSpeedCounterChangedThreshold2=LaneIn.uSpeedCounterChangedThreshold2;

		p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].uLaneID=0;
		memset((void *)&p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].uTrackParams,0,sizeof(Uint16)*20);
		memset((void *)&p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].uReserved0,0,sizeof(Uint16)*30);
		memset((void *)&p_pDetectCfgSeg->uSegData[0].SpeedEachLane[i].uReserved1,0,sizeof(Uint16)*20);
		for(int j = 0; j < 2; j++)
		{
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].DetectInSum=0;
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].calarflag=0;
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].DetectOutSum=0;
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].uVehicleHeight=0;
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].uVehicleLength=0;
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].uVehicleSpeed=0;
			p_outbuf->uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].calarflag=0;
			p_outbuf->uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].DetectInSum=0;
			p_outbuf->uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].DetectOutSum=0;
			p_outbuf->uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].uVehicleHeight=0;
			p_outbuf->uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].uVehicleLength=0;
			p_outbuf->uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].uVehicleSpeed=0;
		}
	}
	//p_pDetectCfgSeg->uSegData[0].uDayNightJudgeMinContiuFrame=15;
	//p_pDetectCfgSeg->uSegData[0].uComprehensiveSens=60;
	//p_pDetectCfgSeg->uSegData[0].uDetectSens1=20;
	//p_pDetectCfgSeg->uSegData[0].uDetectSens2=20;
	//p_pDetectCfgSeg->uSegData[0].uStatisticsSens1=30;//30
	//p_pDetectCfgSeg->uSegData[0].uStatisticsSens2=3;//3
	//p_pDetectCfgSeg->uSegData[0].uSobelThreshold=3;

	p_pDetectCfgSeg->uSegData[0].uDayNightJudgeMinContiuFrame=LaneIn.uDayNightJudgeMinContiuFrame;
	p_pDetectCfgSeg->uSegData[0].uComprehensiveSens=LaneIn.uComprehensiveSens;
	p_pDetectCfgSeg->uSegData[0].uDetectSens1=LaneIn.uDetectSens1;
	p_pDetectCfgSeg->uSegData[0].uDetectSens2=LaneIn.uDetectSens2;
	p_pDetectCfgSeg->uSegData[0].uStatisticsSens1=LaneIn.uStatisticsSens1;//30
	p_pDetectCfgSeg->uSegData[0].uStatisticsSens2=LaneIn.uStatisticsSens2;//3
	p_pDetectCfgSeg->uSegData[0].uSobelThreshold=LaneIn.uSobelThreshold;
	//printf("uEnvironment0 is %d/t",LaneIn.uEnvironment);
	//printf("uEnvironmentStatus0 is %d/n",LaneIn.uEnvironmentStatus);
	p_pDetectCfgSeg->uSegData[0].uEnvironment=LaneIn.uEnvironment;//锟斤拷锟斤拷锟斤拷锟�
	p_pDetectCfgSeg->uSegData[0].uEnvironmentStatus=LaneIn.uEnvironmentStatus;//锟狡昏开始
        p_pDetectCfgSeg->uSegData[0].base_length=LaneIn.base_length;
        p_pDetectCfgSeg->uSegData[0].near_point_length=LaneIn.near_point_length;
	/*************************************************************************************************************/
	//memset((void *)&pParams,0,sizeof(pParams));


	/*************************************************************************************************************/



	/*************************************************************************************************************/
	ChNum=0;
	flag_ini=ArithInit(ChNum, p_pCfgHeader, p_pDetectCfgSeg, pCfgs, pParams);//,m_ptend,LineUp//ARM->DSP锟斤拷锟斤拷
	//printf("190 is ok\t");
	if(flag_ini) return TRUE;
	else return FALSE;

}

typedef struct test_cam{
	unsigned int frame_count;
	long long start_time;
	unsigned int total_time;
	unsigned int alg_tota_time;
}test_cam_t;

test_cam_t came_test[20] = {0};

CIVD_SDK_API unsigned int transform_Proc_DSP_VC(int index, unsigned char  *pInFrameBuf,
		unsigned char *pInuBuf,unsigned char *pInvBuf,int nWidth,\
		int nHeight, int hWrite,RESULTMSG *p_outbuf,m_args *p_arg)//,CPoint LineUp[],CPoint m_ptend[]
{
	ALGPARAMS *pParams=p_arg->pParams;
	ALGCFGS *pCfgs=p_arg->pCfgs;
//	 RESULTMSG *p_outbuf=p_arg->p_outbuf;
	OUTBUF *outBuf=p_arg->p_outbuf;
	Uint16 ChNum=0;
	Int32 outSize;
	SPEEDCFGSEG    *p_pDetectCfgSeg=&p_arg->pDetectCfgSeg;
	CFGINFOHEADER  *p_pCfgHeader=&p_arg->pCfgHeader;
	ChNum=0;
	outSize=sizeof(RESULTMSG);
	//printf("start to ArithProc");
	struct timeval start_time, end_time1,end_time2;
	ArithProc(ChNum, pInFrameBuf, pInuBuf, pInvBuf, nWidth, nHeight, p_outbuf, outSize, pCfgs, pParams);
	//printf("232 is ok\t");
	//printf("ArithProc is ok");

	for(Uint8 i=0;i<pCfgs->ResultMsg.uResultInfo.LaneSum;i++)
	{
		for(Uint8 j = 0; j < 2; j++)//0为前线圈 1为后线圈
		{
			outBuf->CoilAttribute[i][j].DetectInSum=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].DetectInSum;
			outBuf->CoilAttribute[i][j].DetectOutSum=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].DetectOutSum;
			outBuf->CoilAttribute[i][j].calarflag=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].calarflag;
			outBuf->CoilAttribute[i][j].uVehicleHeight=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].uVehicleHeight;
			outBuf->CoilAttribute[i][j].uVehicleLength=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].uVehicleLength;
			outBuf->CoilAttribute[i][j].uVehicleSpeed=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].uVehicleSpeed;
			outBuf->CoilAttribute[i][j].uVehicleType = pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].uVehicleType;
			outBuf->CoilAttribute[i][j].uVehicleHeadtime = pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.CoilAttribute[j].uVehicleHeadtime;
		}
		//for (int j=0;j<5;j++)
		//{
		//	outBuf->AlarmLineflag[i][j]=pCfgs->sptimes[j];//pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.AlarmLineflag;
		//}
		outBuf->LineUp[i][0]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.LineUp[0];
		outBuf->LineUp[i][1]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.LineUp[1];
		outBuf->uLastVehicleLength[i]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uLastVehicleLength;
		outBuf->IsCarInTail[i]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.IsCarInTailFlag;
		outBuf->getQueback_flag[i]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.getQueback_flag;//txl,20160104
		outBuf->DetectRegionVehiSum[i]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uDetectRegionVehiSum;//区域车辆数
		outBuf->QueLine[i][0] = pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.QueLine[0];//排队点
		outBuf->QueLine[i][1] = pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.QueLine[1];
		outBuf->uVehicleQueueLength[i]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uVehicleQueueLength;//排队长度
		outBuf->uQueueHeadDis[i]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uQueueHeadDis;//队首距离
	    outBuf->uQueueTailDis[i]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uQueueTailDis;//队尾距离
	    outBuf->uQueueVehiSum[i]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uQueueVehiSum;//通道排队数量
	    outBuf->uVehicleDensity[i]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uVehicleDensity;//空间占有率
	    outBuf->uVehicleDistribution[i]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uVehicleDistribution;//车辆分布情况
	    outBuf->uHeadVehiclePos[i]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uHeadVehiclePos;//头车位置
	    outBuf->uHeadVehicleSpeed[i]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uHeadVehicleSpeed;//头车速度
	    outBuf->uLastVehiclePos[i]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uLastVehiclePos;//末车位置
	    outBuf->uLastVehicleSpeed[i]=pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo1.uLastVehicleSpeed;//末车速度
		outBuf->uAverVehicleSpeed[i] = (outBuf->DetectRegionVehiSum[i] == 0)? 0 : outBuf->CoilAttribute[i][0].uVehicleSpeed;//平均速度
		printf("coil 1 region num = %d,[%d,%d,%d,%d], coil 2 [%d,%d,%d,%d] head =%d,tail = %d,density =%d\n",outBuf->DetectRegionVehiSum[i], outBuf->CoilAttribute[i][0].DetectOutSum,outBuf->CoilAttribute[i][0].uVehicleLength,outBuf->CoilAttribute[i][0].uVehicleSpeed,outBuf->CoilAttribute[i][0].uVehicleHeadtime,outBuf->CoilAttribute[i][1].DetectOutSum,outBuf->CoilAttribute[i][1].uVehicleLength,outBuf->CoilAttribute[i][1].uVehicleSpeed,outBuf->CoilAttribute[i][1].uVehicleHeadtime,outBuf->uHeadVehiclePos[i],outBuf->uLastVehiclePos[i],outBuf->uVehicleDensity[i]);
		//printf("laneID = %d,car_in = %d, length =%d , length1 = %d,vehicle_num = %d\n",i,outBuf->DetectInSum[i],outBuf->uVehicleQueueLength[i] ,outBuf->uVehicleQueueLength1[i],outBuf->DetectRegionVehiSum[i]);
//        std::cout<<outBuf->DetectRegionVehiSum[i]<<endl;
//        std::cout<< outBuf->QueLine[i][0].x<<","<<outBuf->QueLine[i][0].y<<endl;
//        std::cout<<outBuf->DetectRegionVehiSum[i]<<endl;
	}
	outBuf->udetNum =pCfgs->ResultMsg.uResultInfo.udetNum;
	for(int i = 0; i< pCfgs->ResultMsg.uResultInfo.udetNum; i++)
	{
		outBuf->udetBox[i] = pCfgs->ResultMsg.uResultInfo.udetBox[i];
		//printf("[%d,%d,%d,%d]\n",pCfgs->ResultMsg.uResultInfo.udetBox[i].x,pCfgs->ResultMsg.uResultInfo.udetBox[i].y,pCfgs->ResultMsg.uResultInfo.udetBox[i].width,pCfgs->ResultMsg.uResultInfo.udetBox[i].height);
	}

    outBuf->udetPersonNum =pCfgs->ResultMsg.uResultInfo.udetPersonNum;//行人数
	for(int i = 0; i< pCfgs->ResultMsg.uResultInfo.udetPersonNum; i++)//行人框
	{
		outBuf->udetPersonBox[i]  = pCfgs->ResultMsg.uResultInfo.udetPersonBox[i];

        //printf( "#########################udetPersonNum: %d ******################# \n", pCfgs->ResultMsg.uResultInfo.udetPersonNum);
	}
	outBuf->udetStatPersonNum = pCfgs->ResultMsg.uResultInfo.udetStatPersonNum;
	for(int i = 0; i < pCfgs->uDetectRegionNum; i++)
	{
		memcpy(outBuf->uPersonDirNum[i], pCfgs->uPersonDirNum[i], MAX_DIRECTION_NUM * sizeof(Uint16));//分方向行人数
	}
	memcpy(outBuf->uPersonRegionNum, pCfgs->uRegionPersonNum, MAX_REGION_NUM * sizeof(Uint16));//区域行人数
	outBuf->fuzzyflag=pCfgs->fuzzyflag;
	outBuf->visibility=pCfgs->visibility;
	for(Uint8 i=0;i<20;i++)
	{
		outBuf->uDegreePoint[i][0]=pCfgs->degreepoint[i][0];
		outBuf->uDegreePoint[i][1]=pCfgs->degreepoint[i][1];
	}
	//printf("degree = [%d,%d],[%d,%d],[%d,%d]\n",outBuf->uDegreePoint[0][0],outBuf->uDegreePoint[0][1],outBuf->uDegreePoint[1][0],outBuf->uDegreePoint[1][1],outBuf->uDegreePoint[2][0],outBuf->uDegreePoint[2][1]);
	for(Uint8 i=0;i<pCfgs->ResultMsg.uResultInfo.LaneSum;i++)
	{
		outBuf->uActualDetectLength[i]=pCfgs->uActualDetectLength[i];
		outBuf->uActualTailLength[i]=pCfgs->uActualTailLength[i];
	}
	//printf("visibility is :%d,",outBuf->visibility);
	//printf("fuzzyflag is :%d,\n",outBuf->fuzzyflag);
	//outBuf->uVehicleHeight[0]=pCfgs->gdWrap;
	//outBuf->DetectInSum[1]=pCfgs->gdSum;
	outBuf->thresholdValue=pCfgs->bAuto;
	//printf("pCfgs->bAuto is %d\n",outBuf->thresholdValue);
	//outBuf->uEnvironmentStatus=pCfgs->ResultMsg.uResultInfo.uEnvironmentStatus; //2019.05.10 by roger

	outBuf->frame++;
    gettimeofday( &end_time2, NULL );
	int alg_ms = (end_time1.tv_sec - start_time.tv_sec)*1000 + (end_time1.tv_usec - start_time.tv_usec)/1000;
	int alg_ms2 = (end_time2.tv_sec - start_time.tv_sec)*1000 + (end_time2.tv_usec - start_time.tv_usec)/1000;
	//printf("ArithProc time =%f,%f\n",(end_time1.tv_sec - start_time.tv_sec) + (end_time1.tv_usec - start_time.tv_usec)/1000000.0,(end_time2.tv_sec - start_time.tv_sec) + (end_time2.tv_usec - start_time.tv_usec)/1000000.0);
	if (came_test[index].start_time == 0) {
		came_test[index].start_time = start_time.tv_sec*1000 + start_time.tv_usec/1000;
	}
	
	came_test[index].total_time += alg_ms2;
	came_test[index].alg_tota_time += alg_ms;
	came_test[index].frame_count++;
	
	if ( ((end_time2.tv_sec*1000 + end_time2.tv_usec/1000) - came_test[index].start_time) >=1000 ) {
		prt(info, "came[%d] alg time: %d run all time: %d frame: %d .......", index, came_test[index].alg_tota_time/came_test[index].frame_count, came_test[index].total_time/came_test[index].frame_count, came_test[index].frame_count);
		came_test[index].start_time = 0;
		came_test[index].frame_count = 0;
		came_test[index].total_time = 0;
		came_test[index].alg_tota_time = 0;
	}
		
#if 0	

	if ( (alg_ms2 - alg_ms) > 5) { 
		prt(info, "ArithProc[%d] time =%d,%d sa:%d.........", index, alg_ms,alg_ms2, alg_ms2-alg_ms);
	}
	else {
		prt(info, "ArithProc[%d] time =%d,%d\n", index, alg_ms,alg_ms2);
	}
#endif

	return 1;
}

int transform_release_DSP_VC(m_args *args)
{
    alg_mem_free(args);

	return 0;
}
int transform_arg_ctrl_DSP_VC(m_args *args)
{

	ZENITH_SPEEDDETECTOR *pDownSpeedDetect =
			(ZENITH_SPEEDDETECTOR*) args->pDetectCfgSeg.uSegData;
	ALGCFGS *pCfgs=args->pCfgs;
	pCfgs->bAuto = pDownSpeedDetect->uEnvironment; //TRUE;
	pCfgs->CameraCfg.uEnvironmentStatus = pDownSpeedDetect->uEnvironmentStatus;
	if (pCfgs->CameraCfg.uEnvironmentStatus == 1
			|| pCfgs->CameraCfg.uEnvironmentStatus == 3)
		pCfgs->CameraCfg.uSquareThreshold =
				(pCfgs->CameraCfg.uSquareThreshold < 20) ?
						pCfgs->CameraCfg.uSquareThreshold : 20; //dawn dusk

	return 0;
}
