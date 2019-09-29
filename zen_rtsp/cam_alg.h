/*
 * cam_alg.h
 *
 *  Created on: 2016��6��30��
 *      Author: root
 */

#ifndef ALG_H_
#define ALG_H_

#include "m_arith.h"
//#include "param.h"
//#include "protocol.h"
#include "client_obj.h"
#include "g_define.h"
enum{
    TIME_SECTION_NULL,
    DUSK=1,
    NIGHT,
    MORNING,
    DAYTIME
};
enum{
    ALG_NULL,
    ALG_DAYTIME=1,
    ALG_NIGHT,
};
typedef struct AlgParam{
    m_args alg_arg;
    LANEINISTRUCT LaneIn;
    short algNum;
    int alg_index;   //Ĭ���������1   1/0
    int time_section;//1-4   1=>huang hun 2=>wan shang  3=>ling cheng   4=> bai tian  �ڿ�ʼ�����ʱ���Լ�ÿ��10s��ȡһ�Σ����ݸ��㷨����
    int framecount;    //֡��������
    SPEEDCFGSEG    pDetectCfgSeg;
    CFGINFOHEADER  pCfgHeader;
    RESULTMSG outbuf;
    int tick;
}mAlgParam;

typedef struct alg_context{
}m_alg_context;
static int alloc_alg(mAlgParam *algparam,mCamDetectParam *p_camdetectparam,mCamParam *p_cammer);
void release_alg(int index);
int run_alg(int index,unsigned char *y,unsigned char *u,unsigned char *v);
int reset_alg(int index);
int open_alg(int index);
void extern init_alg(int index);


typedef struct coil_rst_info {
	Uint16 veh_len;//车长 单位：m

	int in_car;//dang qian jin che biao zhi 当前进车标识
    long long in_car_time;//dang qian jin che biao zhi 当前进车时间?
    int out_car;// dang qian chu che biao zhi 当前出车标识
    long long out_car_time;// dang qian chu che biao zhi 当前出车时间?
    int exist_flag;
    int last_exist_flag;
	int head_time; //车头时距

    //
    unsigned int stay_ms; //停留时间
    //
    unsigned char obj_type;//车辆类型
    //
    unsigned int at_in_car_time; //入车时间ms
	unsigned int at_out_car_time;//出车时间ms
} coil_rst_info_t;
typedef struct lane_rst_info{
    int no;// che dao bian hao //车道编号
    int ms;// jian ce shi jian(ms) //检测时�?
    Uint16 queue_len;// pai dui chang du //排队长度
    Uint16 queue_head_len;//队首距离
    Uint16 queue_tail_len;//队尾距离
    Uint16 queue_no;//通道排队数量
    Uint16 veh_no;// che liang zong shu//车辆总数
    Uint16 start_pos;// pai dui kai shi wei zhi //车队开始位�?
    Uint16 speed;// shang yi liang che de che su//上一辆车的车�?
    Uint16 ocupation_ratio; //空间占有�?
    Uint16 veh_type;//che liang lei xing //车辆类型
    Uint16 average_speed;//平均速度
    Uint16 locate;//分布情况
    Uint16 head_veh_pos;//头车位置
    Uint16 head_veh_speed;//头车速度
    Uint16 tail_veh_pos;//末车位置
    Uint16 tail_veh_speed;//末车速度
    //Uint16 veh_len;//车长 单位：m

	coil_rst_info_t coils[NANJING_LANE_COIL_MAX]; //线圈统计信息
    int det_status;
}lane_rst_info_t;
typedef struct cam_rst_info{
    lane_rst_info_t lanes[MAX_LANE_NUM];
}cam_rst_info_t;
typedef struct frame_info{
    cam_rst_info_t cams[CAM_MAX];
}frame_info;
typedef struct l_d{
    int exist_duration; //整个停留时间
    int pass_number;    //总经过的车辆�?
    int speed_sum;      //总的速度�?
    int veh_len_sum;    //总的车长�?
    int head_len_sum;   //总的车头�?
    int car_a_sum;//A类车流量
	int car_b_sum;
	int car_c_sum;
	int head_time_sum; //车头时距
}l_d_t;
typedef struct data_60s{
    int data_valid;//when valid is false, accumulate start;when time is up , calculate and turn vaild true , and stop accumulate
    l_d_t lane_data[MAX_LANE_NUM][NANJING_LANE_COIL_MAX];
}data_60s_t;

typedef struct l_c_d{
	unsigned char lane_no;//车道编号
	unsigned char queue_len_max;
	int exist_duration; //整个停留时间
	int car_stop_sum;//停车次数
	unsigned char queue_status; //0--没有排队 1--为有排队
}l_c_d_t;

typedef struct data_300s {
	l_c_d_t lane_data[MAX_LANE_NUM];
}data_300s_t;

extern frame_info info;
extern data_60s_t d60[CAM_MAX];
extern data_300s_t d300[CAM_MAX];

//
void get_person_flow(int index, Uint16 (*p_area_person)[MAX_DIRECTION_NUM], long long *p_density);
void add_person_flow(int index, Uint16 (*p_area_person)[MAX_DIRECTION_NUM], long long *p_density);
//
void send_person_sig(int index,int person_count,int thre);
//
void detector_static_data(int index);


#endif /* INCLUDE_CAM_ALG_H_ */
