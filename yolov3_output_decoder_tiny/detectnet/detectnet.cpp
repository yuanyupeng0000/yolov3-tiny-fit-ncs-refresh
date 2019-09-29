
/*
 * Company:	Systhesis
 * Author: 	Chen
 * Date:	2018/06/04	
 */

#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#include "yolo_layer.h"
#include "image.h"
#include "cuda.h"

#include <caffe/caffe.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <sys/time.h>

using namespace caffe;
using namespace cv;
//using namespace std;

//YOLOV3
const string& model_file = "../ncs/yolov3-tiny-ncs-caffe.prototxt";
const string& weights_file = "../ncs/Jenerated_nolastpooling.caffemodel";


const char* imgFilename = "../ncs/standard_size_416_persons.jpg";
//const char* imgFilename = "/data/darknet/data/0000171.jpg";

uint64_t current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    return te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
}

bool signal_recieved = false;

void sig_handler(int signo)
{
    if( signo == SIGINT ){
	printf("received SIGINT\n");
	signal_recieved = true;
    }
}


int main( int argc, char** argv )
{
    printf("detectnet-camera\n  args (%i):  ", argc);

    for( int i=0; i < argc; i++ )
	printf("%i [%s]  ", i, argv[i]);
	
    printf("\n\n");
    //string model_file = string(argv[1]);
    //string weights_file = string(argv[2]);
    //char* imgFilename = argv[3];

    if( signal(SIGINT, sig_handler) == SIG_ERR )
	printf("\ncan't catch SIGINT\n");

    // Initialize the network.
    Caffe::set_mode(Caffe::GPU);

    /* Load the network. */
    shared_ptr<Net<float> > net;
    net.reset(new Net<float>(model_file, TEST));
    net->CopyTrainedLayersFrom(weights_file);

    printf("num_inputs is %d\n",net->num_inputs());
    printf("num_outputs is %d\n",net->num_outputs());
    CHECK_EQ(net->num_inputs(), 1) << "Network should have exactly one input.";
    CHECK_EQ(net->num_outputs(), 2) << "Network should have exactly two outputs.";
    ///CHECK_EQ(net->num_outputs(), 3) << "Network should have exactly three outputs.";	

    Blob<float> *input_data_blobs = net->input_blobs()[0];
    LOG(INFO) << "Input data layer channels is  " << input_data_blobs->channels();
    LOG(INFO) << "Input data layer width is  " << input_data_blobs->width();
    LOG(INFO) << "Input data layer height is  " << input_data_blobs->height();

    int size = input_data_blobs->channels()*input_data_blobs->width()*input_data_blobs->height();
    float* inputDataCUDA = NULL;
    assert(!cudaMallocManaged(&inputDataCUDA, size*sizeof(float)));

    uint64_t beginDataTime =  current_timestamp();
    //load image
    image im = load_image_color((char*)imgFilename,0,0);
    image sized = letterbox_image(im,input_data_blobs->width(),input_data_blobs->height());
    cuda_push_array(input_data_blobs->mutable_gpu_data(),sized.data,size);

    uint64_t endDataTime =  current_timestamp();

    //YOLOV3 objection detection implementation with Caffe
    uint64_t beginDetectTime =  current_timestamp();

    net->Forward();

    //get the net output
    const std::vector<std::string> &layer_names_vec = net->layer_names();
    for(int i=0; i<layer_names_vec.size(); i++)
    {
        std::string str=layer_names_vec[i];
        std::cout << str << std::endl;
        const shared_ptr <Layer<float>> net_layer = net->layer_by_name(str);
        vector<shared_ptr<Blob<float>>> &blob_ptr_vec = net_layer->blobs();
        for(int j=0; j<blob_ptr_vec.size(); j++)
        {
            shared_ptr<Blob<float>> blob_ptr = blob_ptr_vec[j];
            printf("blob shape:%s\n", blob_ptr->shape_string().c_str());
            //printf("blob count:%d\n", blob_ptr->count(0));
        }
    }
    const std::vector<std::string> &blob_names_vec = net->blob_names();
    for(int i=0; i<blob_names_vec.size(); i++)
    {
        std::string str=blob_names_vec[i];
        std::cout << str << std::endl;
    }
    const vector<shared_ptr<Blob<float>>> &params_vec = net->params();
    for(int j=0; j<params_vec.size(); j++)
    {
        shared_ptr<Blob<float>> blob_ptr = params_vec[j];
        printf("params shape:%s\n", blob_ptr->shape_string().c_str());
        //printf("params count:%d\n", blob_ptr->count(0));
    }

    vector<Blob<float>*> blobs;
    blobs.clear();
    Blob<float>* out_blob1 = net->output_blobs()[1];
    blobs.push_back(out_blob1);
    Blob<float>* out_blob2 = net->output_blobs()[0];
    blobs.push_back(out_blob2);
    ///Blob<float>* out_blob1 = net->output_blobs()[1];
    ///blobs.push_back(out_blob1);
    ///Blob<float>* out_blob2 = net->output_blobs()[2];
    ///blobs.push_back(out_blob2);
    ///Blob<float>* out_blob3 = net->output_blobs()[0];
    ///blobs.push_back(out_blob3);

    printf("output blob1 shape c= %d, h = %d, w = %d\n",out_blob1->channels(),out_blob1->height(),out_blob1->width());
    printf("output blob2 shape c= %d, h = %d, w = %d\n",out_blob2->channels(),out_blob2->height(),out_blob2->width());
    ///printf("output blob3 shape c= %d, h = %d, w = %d\n",out_blob3->channels(),out_blob3->height(),out_blob3->width());

    int nboxes = 0;
    detection *dets = get_detections(blobs,im.w,im.h,&nboxes);
    printf("nboxes=%d\n", nboxes);

    uint64_t endDetectTime = current_timestamp();
    printf("object-detection:  finished processing data operation  (%zu)ms\n", endDataTime - beginDataTime);
    printf("object-detection:  finished processing yolov3 network  (%zu)ms\n", endDetectTime - beginDetectTime);


    //show detection results
    Mat img = imread(imgFilename);

    int i,j;
    for(i=0;i< nboxes;++i){
        char labelstr[4096] = {0};
        int cls = -1;
        //for(j=0;j<80;++j){
        for(j=0;j<6;++j){
            if(dets[i].prob[j] > 0.1){
                if(cls < 0){
                    cls = j;
                }
                printf("%d: %.0f%%\n",cls,dets[i].prob[j]*100);
            }
        }
        if(cls >= 0){
            box b = dets[i].bbox;
            printf("x = %f,y =  %f,w = %f,h =  %f\n",b.x,b.y,b.w,b.h);

            int left  = (b.x-b.w/2.)*im.w;
            int right = (b.x+b.w/2.)*im.w;
            int top   = (b.y-b.h/2.)*im.h;
            int bot   = (b.y+b.h/2.)*im.h;
            rectangle(img,Point(left,top),Point(right,bot),Scalar(0,0,255),1,8,0);
            printf("left = %d,right =  %d,top = %d,bot =  %d\n",left,right,top,bot);
        }
    }

    namedWindow("show",CV_WINDOW_AUTOSIZE);
    imshow("show",img);
    waitKey(0);

    free_detections(dets,nboxes);
    free_image(im);
    free_image(sized);
        

    cudaFree(inputDataCUDA);
    printf("detectnet-camera:  video device has been un-initialized.\n");
    printf("detectnet-camera:  this concludes the test of the video device.\n");
    return 0;
}


