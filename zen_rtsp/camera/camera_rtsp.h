#ifndef __CAMERA_RTSP_H__
#define __CAMERA_RTSP_H__


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/videoio.hpp>
#include <libavformat/avformat.h>


using namespace cv;
bool open_camera(char *path,VideoCapture &cap);
bool get_frame(Mat &fm, VideoCapture cap);

bool get_frame_ffmpeg(AVFormatContext *input_ctx , AVCodecContext *decoder_ctx);
int  open_ffmpeg(char *url, AVFormatContext *input_ctx, AVCodecContext *decoder_ctx);



#endif



