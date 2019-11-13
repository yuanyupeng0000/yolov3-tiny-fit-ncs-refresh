#ifndef INCLUDE_CAM_EVENT_H_
#define INCLUDE_CAM_EVENT_H_
#include <stdlib.h>
#include <pthread.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>

enum MvdEvent{
	OVER_SPEED=1,//����
	REVERSE_DRIVE,//����
	STOP_INVALID,//ͣ��
	NO_PEDESTRIANTION,//����
	DRIVE_AWAY,//ʻ��
	CONGESTION,//ӵ��
	AbANDON_OBJECT,//������
	PERSONFALL = 10,//���˵���
	NONMOTORFALL = 12,//�����г�����
	NON_MOTOR=20,//���зǻ�����
	ACCIDENTTRAFFIC = 21,//��ͨ�¹�
	GREENWAYDROP = 22//�̵�������
};

typedef struct pv_node_value {
	Mat frame;
	void *et_value;
	int cam_index;
}

typedef struct node
{
	char *p_value;
	PNode next;
}Node, *PNode;

//typedef struct node *PNode;

 
typedef struct
{
	PNode front;
	PNode rear;
	int size;
	pthread_mutex_t q_lock;
	pthread_cond_t cond;
}Queue;
 
/*构造一个空队列*/
Queue *InitQueue();
 
/*销毁一个队�?/
void DestroyQueue(Queue *pqueue);
 
/*清空一个队�?/
void ClearQueue(Queue *pqueue);
 
/*判断队列是否为空*/
int IsEmpty(Queue *pqueue);
 
/*返回队列大小*/
int GetSize(Queue *pqueue);
 
/*返回队头元素*/
PNode GetFront(Queue *pqueue, Frame *frame);
 
/*返回队尾元素*/
PNode GetRear(Queue *pqueue, Frame *frame);
 
/*将新元素入队*/
PNode EnQueue(Queue *pqueue,Frame frame);
 
/*队头元素出队*/
PNode DeQueue(Queue *pqueue);
 
/*遍历队列并对各数据项调用visit函数*/
void QueueTraverse(Queue *pqueue,void (*visit)());

void get_names(char *pic_name,char *video_name, int type);

#endif


