#include<iostream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include "cam_event.h"
#include "g_define.h"
#include"Queue.h"

int buffer_num[CAM_MAX] = {0};
Mat buffer_frame[CAM_MAX];
mutex frame_save_lock[CAM_MAX];


void get_names(char *pic_name,char *video_name, int cam_index, int type)
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    sprintf(pic_name, "%d_%d_%lu.png", cam_index, type, tv.tv_sec*1000000 + tv.tv_usec);
    sprintf(video_name, "%d_%d_%lu.mp4", cam_index, type, tv.tv_sec*1000000 + tv.tv_usec, type);
}

#if 0
int save_video(int cam_index,char *f_name, Mat fst)
{
    frame_save_lock[cam_index].lock();
    if (buffer_num[cam_index] < 1) {
        frame_save_lock[cam_index].unlock();
        return -1;
    }

    Mat fst=buffer_frames[cam_index].front();
    cv::VideoWriter recVid(f_name, cv::VideoWriter::fourcc('H', '2', '6', '4'), 15,  cv::Size(fst.cols, fst.rows));
    for(int i = 0; i < buffer_num[cam_index]; i++) {
        recVid.write(buffer_frame[i]);
        buffer_frame[i].release();
    }

    frame_save_lock[cam_index].unlock();
    return 1;
}


static inline VdPoint add_point_offset(VdPoint p_ori,VdPoint p_offset)
{
     return VdPoint(p_ori.x+p_offset.x,p_ori.y+p_offset.y);
}


#endif


#if 0
void insert_picture(Mat frame,VdPoint *outline,int type,vector <VdPoint> region,string pic_path)
{

   string str;

   switch(type){
   case OVER_SPEED:
      // str.append("OVER_SPEED");
       str.append("超速");
       break;
   case REVERSE_DRIVE:
       //str.append("REVERSE_DRIVE");
       str.append("逆道行驶");
       break;
   case STOP_INVALID:
       //str.append("STOP_INVALID");
       str.append("违法停车");
       break;
   case NO_PEDESTRIANTION:
       //str.append("NO_PEDESTRIANTION");
       str.append("行人");
       break;
   case DRIVE_AWAY:
       //str.append("DRIVE_AWAY");
       str.append("驶离");
       break;
   case CONGESTION:
       //str.append("CONGESTION");
       str.append("拥堵");
       break;
   case AbANDON_OBJECT:
       //str.append("AbANDON_OBJECT");
       str.append("抛洒物");
       break;
   case NON_MOTOR:
       //str.append("NON_MOTOR");
       str.append("禁行非机动车");
       break;
   case ACCIDENTTRAFFIC:
       str.append("事故");
       break;
   default:break;

   }

   CvxText text("./simhei.ttf"); //指定字体
   cv::Scalar size1{ 40, 0.5, 0.1, 0 }; // (字体大小, 无效�? 字符间距, 无效�?}

   text.setFont(nullptr, &size1, nullptr, 0);
   wchar_t *w_str;
   char *pstr = (char *)str.data();
   ToWchar(pstr,w_str);
   text.putText(frame, w_str, Point(100,130),Scalar(0,255,255));

   VdRect rct= vers_2_rect(region);
   VdPoint offset(rct.x,rct.y);

   if(outline != NULL){
       for(int i=0;i< 4;i++){
           VdPoint start=add_point_offset(outline[i],offset);
           VdPoint end=add_point_offset(outline[i+1],offset);
           line(frame, Point(start.x,start.y),Point( end.x,end.y), Scalar(255, 255 ,0), 3, 8, 0 );
       }
       VdPoint start=add_point_offset(outline[0],offset);
       VdPoint end=add_point_offset(outline[3],offset);
       line(frame, Point(start.x,start.y),Point( end.x,end.y), Scalar(255, 255 ,0), 3, 8, 0 );

   }

   imwrite(pic_path,frame);

}

//保存行人属性图�?
void insert_man_picture(Mat frame,VdRect &region,string pic_path)
{
   if ( (region.x +region.w) > frame.cols || (region.y + region.h) > frame.rows ) {
       prt(info, "region error!");
       prt(info, "rows: %d cols: %d x: %d y: %d w: %d H: %d", frame.rows, frame.cols, region.x, region.y, region.w, region.h);
       return;
   }

   Rect rect(region.x, region.y, region.w, region.h);
   Mat man_image = frame(rect);
   imwrite(pic_path,man_image);
}

void *pic_video_handle(void *data)
{
	char pic_name[FILENAME_SIZE]; 
	char vedio_name[FILENAME_SIZE];
	while(1) {
		PNode p_node = GetFront(data);
		get_names(pic_name, vedio_name, 1);
		insert_picture(p_node.frame, );
	}
}

 
/*构造一个空队列*/
Queue *InitQueue()
{
	Queue *pqueue = (Queue *)malloc(sizeof(Queue));
	if(pqueue!=NULL)
	{
		pqueue->front = NULL;
		pqueue->rear = NULL;
		pqueue->size = 0;
		pthread_mutex_init(&pqueue->q_lock, NULL);		
		pthread_cond_init(&pqueue->cond, NULL);
	}
	return pqueue;
}
 
/*销毁一个队列*/
void DestroyQueue(Queue *pqueue)
{
	if(!pqueue)
		return;
	ClearQueue(pqueue);
	pthread_mutex_destroy(&pqueue->q_lock);
	pthread_cond_destroy(&pqueue->cond);
	free(pqueue);
	pqueue = NULL;
}
 
/*清空一个队列*/
void ClearQueue(Queue *pqueue)
{
	while(!IsEmpty(pqueue)) {
		DeQueue(pqueue);
	}
 
}
 
/*判断队列是否为空*/
int IsEmpty(Queue *pqueue)
{
	if(pqueue->front==NULL&&pqueue->rear==NULL&&pqueue->size==0)
		return 1;
	else
		return 0;
}
 
/*返回队列大小*/
int GetSize(Queue *pqueue)
{
	return pqueue->size;
}
 
/*返回队头元素*/
PNode GetFront(Queue *pqueue)
{
	pthread_mutex_lock(&pqueue->q_lock);
	/*
	if(!IsEmpty(pqueue))
	{
		*frame = pqueue->front->frame;
	}else {
		pthread_cond_wait(&pqueue->cond, &pqueue->q_lock);
	}*/
	while(IsEmpty(pqueue))
		pthread_cond_wait(&pqueue->cond, &pqueue->q_lock);
	//*frame = pqueue->front->frame;
	
	pthread_mutex_unlock(&pqueue->q_lock);
	return pqueue->front;//---->此处有bug，队列为空时，在锁释放后，pqueue->front可能被入队操作赋值，出现frame等于NULL，而pqueue->front不等于NULL
}
 
/*返回队尾元素*/
 
PNode GetRear(Queue *pqueue)
{
	//if(!IsEmpty(pqueue)) {
	//	*frame = pqueue->rear->frame;
	//}
	return pqueue->rear;
}
 
/*将新元素入队*/
PNode EnQueue(Queue *pqueue, char *p_value)
{
	PNode pnode = (PNode)malloc(sizeof(Node));
	if(pnode != NULL) {
		pnode->p_value = p_value;
		pnode->next = NULL;
		
		pthread_mutex_lock(&pqueue->q_lock);
		if(IsEmpty(pqueue)) {
			pqueue->front = pnode;
		} else {
			pqueue->rear->next = pnode;
		}
		pqueue->rear = pnode;
		pqueue->size++;
		pthread_cond_signal(&pqueue->cond);
		pthread_mutex_unlock(&pqueue->q_lock);
	}
	return pnode;
}
 
/*队头元素出队*/
PNode DeQueue(Queue *pqueue)
{
	PNode pnode = pqueue->front;
	pthread_mutex_lock(&pqueue->q_lock);
	if(!IsEmpty(pqueue)) {
		pqueue->size--;
		pqueue->front = pnode->next;
		free(pnode);
		if(pqueue->size==0)
			pqueue->rear = NULL;
	}
	pthread_mutex_unlock(&pqueue->q_lock);
	return pqueue->front;
}
 
/*遍历队列并对各数据项调用visit函数*/
void QueueTraverse(Queue *pqueue, void (*visit)())
{
	PNode pnode = pqueue->front;
	int i = pqueue->size;
	while(i--)
	{
		visit(pnode->p_value);
		pnode = pnode->next;
	}
		
}

#endif

