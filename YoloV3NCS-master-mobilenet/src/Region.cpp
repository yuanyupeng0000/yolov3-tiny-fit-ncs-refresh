
#include "Region.h"
#include "Common.h"

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include <string>
//10,25,  20,50,  30,75, 50,125,  80,200,  150, 150 refined-anchors
//10,14,  23,27,  37,58,  81,82,  135,169,  344,319 yolov3-tiny

const std::string objectnames[] = { "aeroplane","bicycle","bird","boat","bottle","bus","car","cat","chair","cow","diningtable","dog","horse","motorbike","person","pottedplant","sheep","sofa","train","tvmonitor"};

const int N = 3;
const float biases1[N*2] = {81,82,  135,169,  344,319};
const float biases2[N*2] = {21,27,  34,52,  86,100};
//const float biases1[N*2] = {50,125,  80,200,  150,150};
//const float biases2[N*2] = {10,25,  20,50,  30,75};
//const std::string objectnames[] = {"bus","car", "truck", "motorbike", "bicycle","person"};

Region::Region()
{
	initialized = false;
	totalLength = 0;
}


void Region::Initialize(int c, int h, int w, int size)
{
	totalLength = c * h * w;
	totalObjects = N * h * w;
	output.resize(totalLength);
	boxes.resize(totalObjects);
	s.resize(totalObjects);

	for(int i = 0; i < totalObjects; ++i)
	{
		s[i].index = i;
		s[i].channel = size;
		s[i].prob = &output[5];
	}


	initialized = true;
}


void Region::GetDetections(float* data, int c, int h, int w,
						   int classes, int imgw, int imgh,
						   float thresh, float nms,
						   int blockwd,
						   std::vector<DetectedObject> &objects)
{
	objects.clear();

	int size = 4 + classes + 1;
	//if(!initialized)
	{
		Initialize(c, h, w, size);
	}

	if(!initialized)
	{
		printf("Fail to initialize internal buffer!\n");
		return ;
	}

	int i,j,k;

	transpose(data, &output[0], size*N, w*h);

	// Initialize box, scale and probability
	for(i = 0; i < h*w*N; ++i)
	{
		int index = i * size;
		//Box
		int n = i % N;
		int row = (i/N) / w;
		int col = (i/N) % w;

		boxes[i].x = (col + logistic_activate(output[index + 0])) / blockwd; //w;
		boxes[i].y = (row + logistic_activate(output[index + 1])) / blockwd; //h;
                if(blockwd == 13)
                {
                    boxes[i].w = exp(output[index + 2]) * biases1[2*n]   / 416; //w;
                    boxes[i].h = exp(output[index + 3]) * biases1[2*n+1] / 416; //h;
                }
                else if(blockwd == 26)
                {
                    boxes[i].w = exp(output[index + 2]) * biases2[2*n]   / 416; //w;
                    boxes[i].h = exp(output[index + 3]) * biases2[2*n+1] / 416; //h;
                }
                if(blockwd == 10)
                {
                    boxes[i].w = exp(output[index + 2]) * biases1[2*n]   / 416; //w;
                    boxes[i].h = exp(output[index + 3]) * biases1[2*n+1] / 416; //h;
                }
                else if(blockwd == 20)
                {
                    boxes[i].w = exp(output[index + 2]) * biases2[2*n]   / 416; //w;
                    boxes[i].h = exp(output[index + 3]) * biases2[2*n+1] / 416; //h;
                }
                else if(blockwd == 9)
                {
                    boxes[i].w = exp(output[index + 2]) * biases1[2*n]   / 288; //w;
                    boxes[i].h = exp(output[index + 3]) * biases1[2*n+1] / 288; //h;
                }
                else if(blockwd == 18)
                {
                    boxes[i].w = exp(output[index + 2]) * biases2[2*n]   / 288; //w;
                    boxes[i].h = exp(output[index + 3]) * biases2[2*n+1] / 288; //h;
                }

                else if(blockwd == 11)
                {
                    boxes[i].w = exp(output[index + 2]) * biases1[2*n]   / 352; //w;
                    boxes[i].h = exp(output[index + 3]) * biases1[2*n+1] / 352; //h;
                }
                else if(blockwd == 22)
                {
                    boxes[i].w = exp(output[index + 2]) * biases2[2*n]   / 352; //w;
                    boxes[i].h = exp(output[index + 3]) * biases2[2*n+1] / 352; //h;
                }
		//Scale
		output[index + 4] = logistic_activate(output[index + 4]);

		//Class Probability
		for(int i=1; i<classes+1; i++)
		{
		    output[index + 4 + i] = logistic_activate(output[index + 4 + i]);
		}
                
		///softmax(&output[index + 5], classes, 1, &output[index + 5]);
		for(j = 0; j < classes; ++j)
		{
                    output[index+5+j] *= output[index+4]; //--classprob = objectnessprob * classprob
		    if(output[index+5+j] < thresh) output[index+5+j] = 0;
		}
	}

	//nms
	for(k = 0; k < classes; ++k)
	{
		for(i = 0; i < totalObjects; ++i)
		{
			s[i].iclass = k;
		}
		qsort(&s[0], totalObjects, sizeof(indexsort), indexsort_comparator);
		for(i = 0; i < totalObjects; ++i){
			if(output[s[i].index * size + k + 5] == 0) continue;
			ibox a = boxes[s[i].index];
			for(j = i+1; j < totalObjects; ++j){
				ibox b = boxes[s[j].index];
				if (box_iou(a, b) > nms){
					output[s[j].index * size + 5 + k] = 0;
				}
			}
		}
	}

	// generate objects
	for(i = 0, j = 5; i < totalObjects; ++i, j += size)
	{
		int iclass = max_index(&output[j], classes);

		float prob = output[j+iclass];

		if(prob > thresh)
		{
			ibox b = boxes[i];

                        //printf("%f %f %f %f\n", b.x, b.y, b.w, b.h);

			int left  = (b.x-b.w/2.)*imgw;
			int right = (b.x+b.w/2.)*imgw;
			int top   = (b.y-b.h/2.)*imgh;
			int bot   = (b.y+b.h/2.)*imgh;
                        
                        //printf("%d %d\n", right-left, bot-top);

			if(left < 0) left = 0;
			if(right > imgw-1) right = imgw-1;
			if(top < 0) top = 0;
			if(bot > imgh-1) bot = imgh-1;


			DetectedObject obj;
			obj.left = left;
			obj.top = top;
			obj.right = right;
			obj.bottom = bot;
			obj.confidence = prob;
			obj.objType = iclass;
			obj.name = objectnames[iclass];
			objects.push_back(obj);
		}
	}

	return ;
}
