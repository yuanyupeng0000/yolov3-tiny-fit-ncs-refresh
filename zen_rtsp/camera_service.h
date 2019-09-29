/*
 * camera_service.h
 *
 *  Created on: 2016年9月9日
 *      Author: root
 */

#ifndef CAMERA_SERVICE_H_
#define CAMERA_SERVICE_H_
#include <pthread.h>
#include "g_define.h"
#include "cam_net.h"

enum{
    CAM_RUNNING,
    CAM_STOPED
};


enum{
	CAMERA_CONTROL_NULL,
	CAMERA_CONTROL_OPEN,
	CAMERA_CONTROL_CLOSE,
	CAMERA_CONTROL_RESET,
	CAMERA_CONTROL_SET_NTP,
	CAMERA_CONTROL_RESET_ALG
};


typedef struct camera_info {
    unsigned char exit_flag;
    int updated;
    int index;
    unsigned char vtype;
//	int stop_run;//TODO: lock this
    pthread_mutex_t run_lock;
    int open_flg;
    unsigned char open_alg;
    int cam_running_state;
	int cam_pre_state;
	unsigned char cam_no_work_cnt;
	unsigned char fuzzy_flag;
	unsigned char visible_flag;
    int cmd;
    unsigned char ip[16];
    unsigned char name[16];
    unsigned char passwd[16];
    int port;
    pthread_mutex_t cmd_lock;
    m_cam_context * p_cam;
#ifdef USE_FILE
    m_h264_file_common h264_file_common;
#endif
#ifdef PLAY_BACK
    m_gl_common gl_common;
#endif
    m_timed_func_data *p_data;
    unsigned char *oubuf;
    unsigned char *oubufu;
    unsigned char *oubufv;
    int watchdog_value;
    int watchdog_frames;
    int watchdog_alg_frames;
	unsigned int run_time;
	unsigned int all_frames;
	unsigned int all_times;
    unsigned int frame_num;
	unsigned char zero_frame_cnt;

	long long per_second;
    /*
    unsigned char ybuf[640*480];
    unsigned char ubuf[640*480/4];
    unsigned char vbuf[640*480/4];
    */

    unsigned char ybuf[FRAME_COLS*FRAME_ROWS];
    unsigned char ubuf[FRAME_COLS*FRAME_ROWS/4];
    unsigned char vbuf[FRAME_COLS*FRAME_ROWS/4];
    /*
    unsigned char alg_ybuf[FRAME_COLS*FRAME_ROWS];
    unsigned char alg_ubuf[FRAME_COLS*FRAME_ROWS/4];
    unsigned char alg_vbuf[FRAME_COLS*FRAME_ROWS/4];
    */
    unsigned char *alg_ybuf;
    unsigned char *alg_ubuf;
    unsigned char *alg_vbuf;
    pthread_mutex_t frame_lock;
    pthread_mutex_t frame_lock_ex;
    pthread_t process_thread;
    pthread_t rtsp_thread;
    //pthread_t watchdog_thread;
} m_camera_info;

#define DATA_SIZE 3
typedef struct yuv_list {
	unsigned char ybuf[DATA_SIZE][FRAME_COLS*FRAME_ROWS];
    unsigned char ubuf[DATA_SIZE][FRAME_COLS*FRAME_ROWS/4];
    unsigned char vbuf[DATA_SIZE][FRAME_COLS*FRAME_ROWS/4];
	unsigned char valid[DATA_SIZE];
	unsigned short i_read;
	unsigned short i_write;
	unsigned short total_num;
	pthread_mutex_t data_lock;
} yuv_list_t;


//
//#include "cam_net.h"
//
//#include "cam_codec.h"
//#include "glplayer.h"
//#include "h264_stream_file.h"
///*
//
// * 代表一个相机的全部资源或者信息
// * */
//typedef struct camera_info {
//	int index;
//	int stop_run;//TODO: lock this
//	int open_flg;
//	int cam_state;
//	int cmd;
//	unsigned char ip[16];
//	unsigned char name[16];
//	unsigned char passwd[16];
//	int port;
//	pthread_mutex_t cmd_lock;
//	m_cam_context * p_cam;
//
//#ifdef USE_FILE
//	m_h264_file_common h264_file_common;
//#endif
//#ifdef PLAY_BACK
//	m_gl_common gl_common;
//#endif
////	m_codec_common codec_common;
//	m_timed_func_data *p_data;
//	unsigned char *oubuf;
//	unsigned char *oubufu;
//	unsigned char *oubufv;
//} m_camera_info;
//

void camera_service_init();
int camera_ctrl(int cmd, int index,int blocked, void *data);
int camera_open(int index,char ip[], int port, char username[],char passwd[], unsigned char vtype);
int camera_close(int index);
int get_cam_running_state(int index);
//
void get_camera_network(int index, void *cam_info);
bool set_camera_network(int index, char *ip, char *name, char *passwd, int port);
//
void init_watchdog();
void *watchdog_func(void *data);
//
void init_yuv_list();
bool get_yuv_item(int index, unsigned char *p_ybuf, unsigned char *p_ubuf, unsigned char *p_vbuf);
void set_yuv_item(int index, unsigned char *p_ybuf, unsigned char *p_ubuf, unsigned char *p_vbuf);

#endif /* CAMERA_SERVICE_H_ */
