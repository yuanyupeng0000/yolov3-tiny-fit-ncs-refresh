/*
 * author ryder jia
 * date 2016??10??11??
 * File introduction: ??????????????
 * service init?????????
 * ??????open??close??control????????????????????????????????????
 * ???????????????process fun
 * ????????????????????
 */
#include <sys/time.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include "camera_service.h"
#include "cam_net.h"
#include "cam_alg.h"
#include "cam_net.h"
#include "cam_codec.h"
#include "glplayer.h"
#include "client_net.h"
#include "camera_service.h"
#include "ini/fvdconfig.h"
#include "h264_stream_file.h"
#include "common.h"
#include "g_fun.h"
#include "camera_rtsp.h"
#include "sig_service.h"

/*

 * ??????????????????????????
 * */
 extern int  sig_fd;
 extern  int stoped;
 extern IVDDevSets g_ivddevsets;
 extern IVDCAMERANUM    g_cam_num;
 extern mCamDetectParam g_camdetect[CAM_MAX];


enum{
    DOG_HUNGRY_STATE,
    DOG_FULL_STATE
};


yuv_list_t cam_yuv_list[CAM_MAX];

void init_yuv_list()
{
    memset(cam_yuv_list, 0, sizeof(yuv_list_t)*CAM_MAX);

    for(int i = 0; i < CAM_MAX; i++) {
        pthread_mutex_init(&cam_yuv_list[i].data_lock, NULL);
    }
}

bool get_yuv_item(int index, unsigned char *p_ybuf, unsigned char *p_ubuf, unsigned char *p_vbuf)
{
    bool ret = false;
    pthread_mutex_lock(&cam_yuv_list[index].data_lock);
    int iread = cam_yuv_list[index].i_read;
    if (cam_yuv_list[index].total_num > 0 && cam_yuv_list[index].valid[iread] > 0) {
        memcpy(p_ybuf, cam_yuv_list[index].ybuf[iread],FRAME_COLS*FRAME_ROWS);
        memcpy(p_ubuf, cam_yuv_list[index].ubuf[iread],FRAME_COLS*FRAME_ROWS/4);
        memcpy(p_vbuf, cam_yuv_list[index].vbuf[iread],FRAME_COLS*FRAME_ROWS/4);
        cam_yuv_list[index].valid[iread] = 0;
        cam_yuv_list[index].i_read = (iread + 1) % DATA_SIZE;
        cam_yuv_list[index].total_num--;
        ret = true;
    }

    pthread_mutex_unlock(&cam_yuv_list[index].data_lock);

    return ret;
}

void set_yuv_item(int index, unsigned char *p_ybuf, unsigned char *p_ubuf, unsigned char *p_vbuf)
{
    pthread_mutex_lock(&cam_yuv_list[index].data_lock);
    //if (cam_yuv_list[index].total_num < DATA_SIZE) {
        int iwrite = cam_yuv_list[index].i_write;
        memcpy(cam_yuv_list[index].ybuf[iwrite], p_ybuf, FRAME_COLS*FRAME_ROWS);
        memcpy(cam_yuv_list[index].ubuf[iwrite], p_ubuf, FRAME_COLS*FRAME_ROWS/4);
        memcpy(cam_yuv_list[index].vbuf[iwrite], p_vbuf, FRAME_COLS*FRAME_ROWS/4);
        cam_yuv_list[index].valid[iwrite] = 1;
        cam_yuv_list[index].i_write = (iwrite + 1) % DATA_SIZE;
        cam_yuv_list[index].total_num++;
        if (cam_yuv_list[index].total_num > DATA_SIZE) {
            cam_yuv_list[index].total_num = DATA_SIZE;
        }
    //}
    pthread_mutex_unlock(&cam_yuv_list[index].data_lock);
}


void *move_pic(void *data)
{
    m_camera_info *p_info=(m_camera_info*)data;
     usleep(20);
    //my_mutex_lock(&p_info->frame_lock);
    pthread_cleanup_push(my_mutex_clean, &p_info->frame_lock);
    pthread_mutex_lock(&p_info->frame_lock);
    prt(info, "move_pic lock.........")
  //   prt(info,"--------------------------------------a-------------------------------->send out");

    run_alg(p_info->index, p_info->ybuf, p_info->ubuf, p_info->vbuf);
    memcpy(p_info->ybuf,p_info->oubuf,FRAME_COLS*FRAME_ROWS);
    memcpy(p_info->ubuf,p_info->oubufu,FRAME_COLS*FRAME_ROWS/4);
    memcpy(p_info->vbuf,p_info->oubufv,FRAME_COLS*FRAME_ROWS/4);
   //  prt(info,"--------------------------------------b-------------------------------->send out");
    //my_mutex_unlock(&p_info->frame_lock);
    pthread_mutex_unlock(&p_info->frame_lock);
    pthread_cleanup_pop(0);
    prt(info, "move_pic unlock.........")
   usleep(20);
}
//#define CAMERA_NUMBER 4
#define CHECK_DURATION 500000//US
#define WATCHDOG_CHECK_DURATION 1000*1000 //1000*1000*10//10s
#define FRAME_DURATION 100000//US
m_camera_info cam_info[CAM_MAX];
/* Function Description
 * name:
 * return:
 * args:
 * comment:???????????????????????????????
 * todo:
 */
void *process_fun(int index, char *data, int size)
{
    FRAME_HEAD	*pFrameHead = (FRAME_HEAD *)data;
    EXT_FRAME_HEAD *exfream = (EXT_FRAME_HEAD *)(data + sizeof(FRAME_HEAD));

    if (FRAME_FLAG_A == pFrameHead->streamFlag)
    {
        return NULL;
    }

    int ret=0;

    m_camera_info *p_info = &cam_info[index];
    p_info->watchdog_value=DOG_FULL_STATE;
    data=data+ sizeof(FRAME_HEAD)+sizeof(EXT_FRAME_HEAD);

    pthread_cleanup_push(my_mutex_clean, &p_info->frame_lock);
    if (0 == pthread_mutex_trylock(&p_info->frame_lock)) {
       h264_decode(index, data-100, size, (unsigned char**)&p_info->alg_ybuf, (unsigned char**)&p_info->alg_ubuf, (unsigned char**)&p_info->alg_vbuf);
       p_info->updated = 1;
       pthread_mutex_unlock(&p_info->frame_lock);
    }
    p_info->watchdog_frames++;
    pthread_cleanup_pop(0);
    cam_info[index].frame_num = 0;

    usleep(30000);

}

void *process_fun_for_rtsp(void* data)
{
    Mat fm;
    int index = *(int*)data;
    unsigned char pYuvBuf[FRAME_COLS*FRAME_ROWS*3/2] = {0};
    unsigned int frame_size = FRAME_COLS*FRAME_ROWS;
    unsigned int frame_uv_size = FRAME_COLS*FRAME_ROWS/4;
    VideoCapture cap;
    //-------test
#if 0
    time_t now = time(NULL);////???1970.1.1?????????time_t
    struct tm * timeinfo = localtime(&now); //????TimeDate,?????????????
    char path[60];
    strftime(path, 60, "%Y_%m_%d_%H_%M_%S", timeinfo);
    char strPath[100];
    sprintf(strPath, "camera%d_%s.avi", index, path);//???????????????????cmdchar??
    VideoWriter writer;
    writer.open(strPath, CV_FOURCC('X', 'V', 'I', 'D'), 25, Size(640, 480), true);//Size(640, 480)//Size(frame.rows, frame.cols)//"cam.avi"
#endif
    //--------end test

    while(true) {

        //pthread_testcancel();
        while(!cam_info[index].open_flg) {
            prt(info, "camera: %d open \n", index);
            if (strlen(g_camdetect[index].other.rtsppath) <1){
                sleep(1);
                continue;
            }

            //if (!open_camera("rtsp://10.10.10.12:554/av0_1", cap)){
            if (!open_camera(g_camdetect[index].other.rtsppath, cap)){
                prt(info, "camera[%d]: %s open faild. \n", index, g_camdetect[index].other.rtsppath);
                sleep(1);
                continue;
            }else {
                cam_info[index].open_flg = 1;
            }
        }

       if (!get_frame(fm, cap)){
            prt(info, "cam[%d] get frame faied. num: %d", index, cam_info[index].frame_num);
            usleep(2000);
            if (cam_info[index].frame_num > 2) {
               // cam_info[index].open_flg = 0;
                cam_info[index].frame_num = 0;
            }
            cam_info[index].frame_num++;
            continue;
        }else{
            if ((fm.cols == 0 || fm.rows == 0) ){
                prt(info,"frame is cols 0");
                continue;
            }
        }

        m_camera_info *p_info = &cam_info[index];
        p_info->watchdog_value=DOG_FULL_STATE;

       // pthread_mutex_lock(&p_info->cmd_lock);

        Mat yuvImg;
        cvtColor(fm, yuvImg, CV_BGR2YUV_I420);
        memcpy(pYuvBuf, yuvImg.data, FRAME_COLS*FRAME_ROWS*3/2*sizeof(unsigned char));

        set_yuv_item(index, pYuvBuf, pYuvBuf + frame_size, pYuvBuf + frame_size + frame_uv_size);
        p_info->watchdog_frames++;
#if 0
        memcpy(p_info->ybuf,pYuvBuf,frame_size);
        memcpy(p_info->ubuf,pYuvBuf + frame_size,frame_uv_size);
        memcpy(p_info->vbuf,pYuvBuf + frame_size + frame_uv_size,frame_uv_size);
#endif

    #if 0
        pthread_cleanup_push(my_mutex_clean, &p_info->frame_lock);

       // pthread_mutex_lock(&p_info->frame_lock);
       if (0 == pthread_mutex_trylock(&p_info->frame_lock)) {
            //prt(info,"cam[%d] for rtsp lock", index);
          //  pthread_mutex_lock(&p_info->frame_lock);
          #if 0
            if(!p_info->updated){
                pthread_mutex_lock(&p_info->frame_lock);
                //prt(info, "process_fun_for_rtsp lock.........")
                memcpy(p_info->alg_ybuf, p_info->ybuf, frame_size);
                memcpy(p_info->alg_ubuf, p_info->ubuf, frame_uv_size);
                memcpy(p_info->alg_vbuf, p_info->vbuf, frame_uv_size);
                p_info->updated = 1;
                //writer.write(fm);
               // imwrite("test.jpg", fm);
               pthread_mutex_unlock(&p_info->frame_lock);
            }
          #endif

            memcpy(p_info->ybuf,pYuvBuf,frame_size);
            memcpy(p_info->ubuf,pYuvBuf + frame_size,frame_uv_size);
            memcpy(p_info->vbuf,pYuvBuf + frame_size + frame_uv_size,frame_uv_size);
            p_info->alg_ybuf = p_info->ybuf;
            p_info->alg_ubuf = p_info->ubuf;
            p_info->alg_vbuf = p_info->vbuf;
            p_info->updated = 1;
            pthread_mutex_unlock(&p_info->frame_lock);
            p_info->watchdog_frames++;

        }

        pthread_cleanup_pop(0);
#endif
        cam_info[index].frame_num = 0;

       usleep(35000);

       // pthread_mutex_unlock(&p_info->cmd_lock);
   }
}


int get_cmd(int index)
{
    return cam_info[index].cmd;
}
void set_cmd(int cmd, int index)
{
    cam_info[index].cmd = cmd;
}
void *file_process_fun(int *index, char *data, int size)
{
    m_camera_info *p_info = (m_camera_info *) data;
    process_fun(*index, data, size);
//	start_detached_func((void *)real_process_fun,data);
}
#include "cam_alg.h"


void *start_process(void *data)
{
    m_camera_info *p_info=(m_camera_info*)data;
    //while(!p_info->exit_flag) {
    while(true) {
        if (!p_info->open_alg) {
            sleep(1);
            //prt(info, "start_process: %d not open_alg", p_info->index);
            continue;
        }

#if 0
       pthread_cleanup_push(my_mutex_clean, &p_info->frame_lock);
       pthread_mutex_lock(&p_info->frame_lock);

       if(p_info->updated == 1){

            int bf_ms = get_ms();
            run_alg(p_info->index, p_info->alg_ybuf, p_info->alg_ubuf, p_info->alg_vbuf);
            int af_ms = get_ms();
            prt(info, "run_alg run time[%d]: %d", p_info->index, af_ms-bf_ms);
            p_info->run_time += (af_ms-bf_ms);
            p_info->all_times += (af_ms-bf_ms);
            p_info->all_frames++;

            p_info->watchdog_alg_frames++;
            p_info->updated = 0;
        }

        pthread_mutex_unlock(&p_info->frame_lock);
        pthread_cleanup_pop(0);
#endif
        #if 0
        long long bf_ms = 0;
        long long af_ms = 0;

        long long bf_ms1 = get_ms();


        if (0 == p_info->per_second) {
            p_info->per_second = bf_ms1;
        }
        #endif
        if ( get_yuv_item(p_info->index, p_info->ybuf, p_info->ubuf, p_info->vbuf) ) {
            pthread_cleanup_push(my_mutex_clean, &p_info->frame_lock);
            pthread_mutex_lock(&p_info->frame_lock);
            //bf_ms = get_ms();
            //prt(info, "camera[%d] call alg start", p_info->index);
            run_alg(p_info->index, p_info->ybuf, p_info->ubuf, p_info->vbuf);
            //prt(info, "camera[%d] call alg end", p_info->index);
            //af_ms = get_ms();

            //p_info->run_time += (af_ms-bf_ms);

            p_info->all_frames++;
            p_info->watchdog_alg_frames++;
            pthread_mutex_unlock(&p_info->frame_lock);
            pthread_cleanup_pop(0);
        }
    #if 0
        int af_ms1 = get_ms();

        p_info->all_times += (af_ms1-bf_ms1);
        if ( ( af_ms1-bf_ms1-(af_ms-bf_ms)) > 5)
            prt(info, "cam[%d] alg: %d process time: %d sa: %d *******************", p_info->index, af_ms-bf_ms,  af_ms1-bf_ms1,  af_ms1-bf_ms1-(af_ms-bf_ms));

        if(p_info->per_second > 0 && (bf_ms1 - p_info->per_second) >= 1000 && p_info->watchdog_alg_frames > 0) {
            prt(info,"watchdog dog for cam %d , fp10s:%d / %d  alg ams: %d run time ams: %d list: %d",
                p_info->index,p_info->watchdog_alg_frames, p_info->watchdog_frames,
                p_info->all_times/p_info->watchdog_alg_frames,
                p_info->all_times/p_info->watchdog_alg_frames, cam_yuv_list[p_info->index].total_num);

            p_info->per_second = 0;
            p_info->watchdog_alg_frames = 0;
            p_info->watchdog_frames = 0;
            p_info->run_time = 0;
            p_info->all_times = 0;
        }
    #endif
        //prt(info, "cam[%d]: client_output start", p_info->index);
        client_output(p_info->index);
        //prt(info, "cam[%d]: client_output end", p_info->index);
        usleep(100);
    }
}

/* Function Description
 * name:
 * return:
 * args:
 * 		index?????????????????????????????????????????ctrl?? ??reset??
 * 		ip port username passwd
 * comment:
 * 		?????????????????????????????????????????????????????????
 * todo:
 */
int camera_open(int index, char ip[], int port, char username[], char passwd[], unsigned char vtype)
{
    int ret=-1;
    static unsigned char open_flg = 0;

    prt(info,"open camera %d vtype: %d",index, vtype);

    if (cam_info[index].open_flg) {
        camera_ctrl(CAMERA_CONTROL_RESET, index, 0, NULL);
        return 0;
    }

    cam_info[index].index = index;
    //prt(info,"ip %s ",ip);
    memset(cam_info[index].ip,0,16);
    memset(cam_info[index].name,0,16);
    memset(cam_info[index].passwd,0,16);
    memcpy(cam_info[index].ip, ip, strlen(ip));
    memcpy(cam_info[index].name, username, strlen(username));
    memcpy(cam_info[index].passwd, passwd, strlen(passwd));
    cam_info[index].port = port;
    cam_info[index].updated = 0;
    cam_info[index].vtype = vtype;

    if (SDK_TYPE == vtype) {
        if ( (0 ==open_flg ) && open_sdk()) {
            prt(info, "err in open sdk");
            return -1;
        } else {
            prt(info, "ok to open sdk");
            open_flg = 1;
        }

        //open_h264_decoder(index); //disable by roger 2019.08.16

        ret=net_open_camera(index, (char *) cam_info[index].ip, cam_info[index].port,
                (char *) cam_info[index].name, (char *) cam_info[index].passwd,
                (void *) process_fun, &cam_info[index]);
    }
    else if(RTSP_TYPE == vtype) {
        prt(info, "open rtsp");
        pthread_create(&cam_info[index].rtsp_thread, NULL, process_fun_for_rtsp, (void *)&index);
        ret = 0;
    }

//	open_camera(cam_info[index].p_cam);
    if(0 == ret){
        open_alg(index);
        cam_info[index].open_alg = 1;
       // open_h264_decoder(index);
        cam_info[index].cam_running_state = CAM_RUNNING;
        //cam_info[index].open_flg = 1;
       // cam_info[index].exit_flag = 0;
        prt(info,"cam_info[%d]: creating thread ............................", index);
        pthread_create( &cam_info[index].process_thread,NULL,start_process, (void *)&cam_info[index]);
        #if 0 //for test
        cam_info[index].watchdog_value=DOG_HUNGRY_STATE;
        m_timed_func_data *p_data_watchdog = regist_timed_func(WATCHDOG_CHECK_DURATION,
                (void *) watchdog_func, (void *) &cam_info[index]);
        start_timed_func(p_data_watchdog);
        cam_info[index].watchdog_thread = p_data_watchdog->handle;
        #endif
    }else{
        cam_info[index].cam_running_state = CAM_STOPED;
        cam_info[index].open_flg = 0;
        cam_info[index].open_alg = 0;
    }

    return 0;
}

int camera_close(int index)
{
    m_camera_info *p_info = (m_camera_info *) &cam_info[index];
    // p_info->exit_flag = 1;
    int open_flg = cam_info[index].open_flg;
    int open_alg = cam_info[index].open_alg;

    cam_info[index].open_alg = 0;
    cam_info[index].open_flg = 0;
    cam_info[index].updated = 0;
    usleep(200000); //????????

    if (SDK_TYPE == cam_info[index].vtype) {
        net_close_camera(index);
    //	close_camera(cam_info[index].p_cam);
        if (cam_info[index].p_cam != NULL) {
            free(cam_info[index].p_cam);
        }

        if (open_flg)
           close_h264_decoder(index);
    }

     prt(info, "cam[%d] rtsp_thread: %u process_thread: %u ",index, cam_info[index].rtsp_thread
    ,cam_info[index].process_thread);

    if ( cam_info[index].rtsp_thread > 0) {
        pthread_cancel(cam_info[index].rtsp_thread);
        pthread_join(cam_info[index].rtsp_thread, NULL);
    }

    if (cam_info[index].process_thread >0) {
        pthread_cancel(cam_info[index].process_thread);
        pthread_join(cam_info[index].process_thread, NULL);
    }

    //unregist_timed_callback(cam_info[index].watchdog_thread);

     prt(info, "camera[%d] close lock", index);
    pthread_mutex_lock(&p_info->frame_lock);
    if (1 == open_alg)
        release_alg(index);
    pthread_mutex_unlock(&p_info->frame_lock);
     prt(info, "camera[%d] close unlock", index);
    usleep(200000);
    cam_info[index].cam_running_state = CAM_STOPED;
    prt(info, "camera[%d] close finish", index);

    return 0;
}

int camera_ctrl(int cmd, int index, int blocked, void *data)
{
    pthread_mutex_lock(&cam_info[index].cmd_lock);
    set_cmd(cmd, index);
    pthread_mutex_unlock(&cam_info[index].cmd_lock);
    return 0;
}

void get_camera_network(int index, void *pinfo)
{
    m_camera_info *p_info = (m_camera_info *)pinfo;
    strncpy((char*)p_info->ip,  (char*)cam_info[index].ip, strlen((char*)cam_info[index].ip) + 1);
    strncpy((char*)p_info->name, (char*)cam_info[index].name, strlen((char*)cam_info[index].name) + 1);
    strncpy((char*)p_info->passwd, (char*)cam_info[index].passwd, strlen((char*)cam_info[index].passwd) + 1);
    p_info->port = cam_info[index].port;
    p_info->vtype = cam_info[index].vtype;
}

bool set_camera_network(int index, char *ip, char *name, char *passwd, int port)
{
    bool ret = false;

    pthread_mutex_lock(&cam_info[index].cmd_lock);

    if ( strcmp((char*)cam_info[index].ip, ip) != 0 ) {
        strncpy((char*)cam_info[index].ip, ip, strlen(ip) + 1);
        ret = true;
    }
    if ( strcmp((char*)cam_info[index].name, name) != 0 ) {
        strncpy((char*)cam_info[index].name, name, strlen(name) + 1);
        ret = true;
    }

    if ( strcmp((char*)cam_info[index].passwd, passwd) != 0 ) {
        strncpy((char*)cam_info[index].passwd, passwd, strlen(passwd) + 1);
        ret = true;
    }

    if ( cam_info[index].port != port ) {
         cam_info[index].port = port;
         ret = true;
    }

    pthread_mutex_unlock(&cam_info[index].cmd_lock);


    return ret;
}

void init_watchdog()
{
    for(int i = 0; i < CAM_MAX; i++) {
        cam_info[i].watchdog_value=DOG_HUNGRY_STATE;
    }
    m_timed_func_data *p_data_watchdog = regist_timed_func(WATCHDOG_CHECK_DURATION,
            (void *) watchdog_func, (void *) NULL);
    start_timed_func(p_data_watchdog);

}

void camera_func(void *data)
{
    static unsigned char reboot_flag = 0;

    m_camera_info *info = (m_camera_info *) data;

        prt(info,"watchdog dog for cam %d , fp10s:%d / %d list: %d",info->index,info->watchdog_alg_frames, info->watchdog_frames, cam_yuv_list[info->index].total_num);
#if 0

        if (info->watchdog_alg_frames > 0)
            prt(info,"watchdog dog for cam %d , run time:%d  frame time: %d ",info->index,info->run_time, info->run_time/info->watchdog_alg_frames);
        if (info->all_frames > 0)
            prt(info,"watchdog dog for cam %d , all time:%d  frame time: %d ",info->index,info->all_times,  info->all_times/info->all_frames);
        info->run_time = 0;
    #endif

        //if(info->watchdog_frames<2&&info->index==0&&stoped==0){

    //if(0){
   // if( (info->watchdog_frames < 2) && (1 == info->open_flg) ){
        //prt(info,"watchdog reboot ");

     //   release_alg(info->index);
    //	system("reboot");
    //}

        if (0 == info->watchdog_alg_frames) {
            info->zero_frame_cnt++;
            if (info->zero_frame_cnt > 120){ //2???????
//				system("reboot");
                prt(info, "reboot");
            }
        }else {
            info->zero_frame_cnt = 0;
        }

    if(info->watchdog_value==DOG_HUNGRY_STATE){
        prt(info,"watch dog for cam %d:camera loop stopped,resetting...",info->index);
        //camera_ctrl(CAMERA_CONTROL_RESET,info->index,1,NULL); 2019.05.28 disable by roger

        if (info->cam_no_work_cnt > 5) {
             camera_ctrl(CAMERA_CONTROL_RESET,info->index,0,NULL);
             info->cam_no_work_cnt = 0;
             cam_info[info->index].open_flg = 0;
        }
        info->cam_running_state = CAM_STOPED;
        info->cam_no_work_cnt++;
    }else{
        info->cam_running_state = CAM_RUNNING;
    //	prt(camera_msg,"watch dog for cam %d:camera running normally",info->index);
    }
    info->watchdog_value = DOG_HUNGRY_STATE;

    info->watchdog_frames=0;
    info->watchdog_alg_frames=0;

    if (g_ivddevsets.autoreset < 8) {

        struct tm *ptr;
        time_t lt;
        lt =time(NULL);
        ptr=localtime(&lt);

        if (g_ivddevsets.autoreset == ptr->tm_wday ) {

            if ( (ptr->tm_hour*60 + ptr->tm_min)  ==  g_ivddevsets.timeset) {
                if (ptr->tm_sec < 30)
                    reboot_flag = 1;
                else if (1 == reboot_flag) //??30???????????????????????????????
                    reboot_cmd();
                  //system("reboot");
            }
        }

    }

    if (PROTO_HUAITONG == g_ivddevsets.pro_type ) {

        if (info->cam_pre_state != info->cam_running_state) { // ?????????
            send_message(0x01, NULL);
        }

        info->cam_pre_state = info->cam_running_state;
    }

}

void *watchdog_func(void *data)
{
    static unsigned short cycle_seconds = 0;

    for(int i=0, j=0; i<CAM_MAX && j < g_cam_num.cam_num; i++) {
        if (1 == g_cam_num.exist[i])
            camera_func(&cam_info[i]);
    }

    //?????????
    if (cycle_seconds >= 300 == g_ivddevsets.overWrite ) { //5 minute
        handle_log_file();
        cycle_seconds = 0;
    }

    cycle_seconds++;
}


int get_cam_running_state(int index)
{
    return cam_info[index].cam_running_state ;
}

/* Function Description
 * name:
 * return:
 * args:data?????????????????????^
 * comment:???????????????????????????????????????
 * todo:
 */
void *camera_main(void *data)
{
    int ret;
    int cmd = CAMERA_CONTROL_NULL;
    for(unsigned short index = 0; index < CAM_MAX && index < g_cam_num.cam_num; index++) {
        pthread_mutex_lock(&cam_info[index].cmd_lock);
        cmd = get_cmd(index);
        pthread_mutex_unlock(&cam_info[index].cmd_lock);

        if (cmd == CAMERA_CONTROL_NULL) {
            //index = (index + 1) % CAM_MAX;
            continue;
        }

        m_camera_info *p_info = (m_camera_info *) &cam_info[index];
        //int index = p_info->index;
       // pthread_mutex_lock(&p_info->cmd_lock);
        switch (cmd) {
        case CAMERA_CONTROL_NULL:
            break;
        case CAMERA_CONTROL_OPEN:
        {
            //prt(info, "open............ camera %d",index);
            m_camera_info camera_info;
            get_camera_network(index, &camera_info);
            camera_open(index, (char *)camera_info.ip, camera_info.port, (char *) camera_info.name, (char *) camera_info.passwd,camera_info.vtype);
            //prt(info, "open............ camera %d end.....",index);
        }
            break;
        case CAMERA_CONTROL_RESET:
            {
                //prt(info, "reseting............ camera %d",index);
                camera_close(index);
                /*
                net_close_camera(index);

                if (1 == p_info->open_flg) {
                   pthread_mutex_lock(&p_info->frame_lock_ex);
                   reset_alg(p_info->index);
                   pthread_mutex_unlock(&p_info->frame_lock_ex);

                   p_info->exit_flag = 1; //??????????
                }

                p_info->open_flg = 0;
                prt(info, "  camera %d reset done",index);
                */
                //m_camera_info camera_info;
                //get_camera_network(index, &camera_info);
                //camera_open(index, (char *)camera_info.ip, camera_info.port, (char *) camera_info.name, (char *) camera_info.passwd, camera_info.vtype);
                  camera_open(index, (char *)g_camdetect[index].other.camIp,(int)g_camdetect[index].other.camPort,(char *)g_camdetect[index].other.username, (char *)g_camdetect[index].other.passwd, g_camdetect[index].other.videotype);   //????????
                /*
                ret=net_open_camera(index, (char *) camera_info.ip,
                        camera_info.port, (char *) camera_info.name,
                        (char *) camera_info.passwd, (void *) process_fun,
                        &cam_info[index]);
                prt(info, "  camera %d opened",index);

                if(ret<0){
                    prt(debug_long,"login %s (port %d) fail",camera_info.ip,camera_info.port);
                }
                */
                // prt(info, "reseting............ camera %d end..",index);
            }
            break;
        case CAMERA_CONTROL_CLOSE:
            {
                camera_close(index);
            }
            break;
        case CAMERA_CONTROL_SET_NTP:
            prt(info, "set cam");
            break;
        case CAMERA_CONTROL_RESET_ALG:
            {
                //prt(info, "reset alg begin");

                if (1 == p_info->open_alg) {
                    p_info->open_alg = 0;
                    usleep(50000);
                    pthread_mutex_lock(&p_info->frame_lock);
                        reset_alg(p_info->index);
                    pthread_mutex_unlock(&p_info->frame_lock);
                    p_info->open_alg = 1;
                }

                //prt(info, "reset alg finish");
            }
            break;
        default:
            break;
        }

        pthread_mutex_lock(&p_info->cmd_lock);
        if (get_cmd(index) != CAMERA_CONTROL_NULL)
            set_cmd(CAMERA_CONTROL_NULL, index);
        pthread_mutex_unlock(&p_info->cmd_lock);

       // index = (index + 1) % CAM_MAX;
   }
}
/* Function Description
 * name:
 * return:
 * args:
 * comment:?????????????????????????????????????????????????????????????????????????cpu?????????????????????
 * ?????????????????????????????????????
 * todo:
 */
void camera_service_init()
{

    int i,j;
#if SDK_TYPE == CAMERA_OPEN_TYPE
    if (open_sdk()) {
        prt(info, "err in open sdk");
    } else {
        prt(info, "ok to open net camera  sdk");
    }
#endif

    for (i = 0, j = 0; i < CAM_MAX && j < g_cam_num.cam_num; i++) {

       if (!g_cam_num.exist[j])
            continue;
        else
            j++;

        init_alg(i);
        cam_info[i].open_flg = 0;
        cam_info[i].index = i;
        cam_info[i].cam_running_state = CAM_STOPED;

        set_cmd(CAMERA_CONTROL_NULL, i);

/*
        pthread_mutex_init(&cam_info[i].cmd_lock, NULL);
        pthread_mutex_init(&cam_info[i].run_lock, NULL);
        pthread_mutex_init(&cam_info[i].frame_lock, NULL);
        pthread_mutex_init(&cam_info[i].frame_lock_ex, NULL);
*/
      //prt(info,"init addr %p,i %d",&cam_info[i].cmd_lock,i);
        #if 0
        m_timed_func_data *p_data = regist_timed_func(CHECK_DURATION,
                (void *) camera_main, (void *) &cam_info[i]);
        start_timed_func(p_data);

        cam_info[i].watchdog_value=DOG_HUNGRY_STATE;
        m_timed_func_data *p_data_watchdog = regist_timed_func(WATCHDOG_CHECK_DURATION,
                (void *) watchdog_func, (void *) &cam_info[i]);
        start_timed_func(p_data_watchdog);
        #endif
    }


    m_timed_func_data *p_data = regist_timed_func(CHECK_DURATION,
        (void *) camera_main, NULL);
    start_timed_func(p_data);
}
