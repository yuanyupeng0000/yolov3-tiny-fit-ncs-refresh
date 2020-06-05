// The contents of this file are in the public domain. See LICENSE_FOR_EXAMPLE_PROGRAMS.txt
/*

    This example program shows how to find frontal human faces in an image and
    estimate their pose.  The pose takes the form of 68 landmarks.  These are
    points on the face such as the corners of the mouth, along the eyebrows, on
    the eyes, and so forth.  
    


    The face detector we use is made using the classic Histogram of Oriented
    Gradients (HOG) feature combined with a linear classifier, an image pyramid,
    and sliding window detection scheme.  The pose estimator was created by
    using dlib's implementation of the paper:
       One Millisecond Face Alignment with an Ensemble of Regression Trees by
       Vahid Kazemi and Josephine Sullivan, CVPR 2014
    and was trained on the iBUG 300-W face landmark dataset (see
    https://ibug.doc.ic.ac.uk/resources/facial-point-annotations/):  
       C. Sagonas, E. Antonakos, G, Tzimiropoulos, S. Zafeiriou, M. Pantic. 
       300 faces In-the-wild challenge: Database and results. 
       Image and Vision Computing (IMAVIS), Special Issue on Facial Landmark Localisation "In-The-Wild". 2016.
    You can get the trained model file from:
    http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2.
    Note that the license for the iBUG 300-W dataset excludes commercial use.
    So you should contact Imperial College London to find out if it's OK for
    you to use this model file in a commercial product.


    Also, note that you can train your own models using dlib's machine learning
    tools.  See train_shape_predictor_ex.cpp to see an example.

    


    Finally, note that the face detector is fastest when compiled with at least
    SSE2 instructions enabled.  So if you are using a PC with an Intel or AMD
    chip then you should enable at least SSE2 instructions.  If you are using
    cmake to compile this program you can enable them by using one of the
    following commands when you create the build project:
        cmake path_to_dlib_root/examples -DUSE_SSE2_INSTRUCTIONS=ON
        cmake path_to_dlib_root/examples -DUSE_SSE4_INSTRUCTIONS=ON
        cmake path_to_dlib_root/examples -DUSE_AVX_INSTRUCTIONS=ON
    This will set the appropriate compiler options for GCC, clang, Visual
    Studio, or the Intel compiler.  If you are using another compiler then you
    need to consult your compiler's manual to determine how to enable these
    instructions.  Note that AVX is the fastest but requires a CPU from at least
    2011.  SSE4 is the next fastest and is supported by most current machines.  
*/


#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <iostream>

#include "face_landmark_detection_ex.hpp"
#include <samples/ocv_common.hpp>
#include <dlib/opencv/cv_image.h>

using namespace dlib;
using namespace std;

// ----------------------------------------------------------------------------------------



static cv::Rect dlibRectangleToOpenCV(dlib::rectangle r)
{
    return cv::Rect(cv::Point2i(r.left(), r.top()), cv::Point2i(r.right() + 1, r.bottom() + 1));
}


static dlib::rectangle openCVRectToDlib(cv::Rect r)
{
    return dlib::rectangle((long)r.tl().x, (long)r.tl().y, (long)r.br().x - 1, (long)r.br().y - 1);
}



std::vector<float> main_(const dlib::shape_predictor& sp, const dlib::frontal_face_detector& ffd,
          const cv::Mat& img_src, const cv::Rect &face_rect)
{
    std::vector<float> landmarks;
    try
    {
        ///for (int i = 2; i < argc; ++i)
        {
            array2d<rgb_pixel> img;
            dlib::cv_image<rgb_pixel> cv_img_dlib(img_src);

            // Make the image larger so we can detect small faces.
            assign_image(img, cv_img_dlib);
            pyramid_up(img);

            // Now tell the face detector to give us a list of bounding boxes
            // around all the faces in the image.
            std::vector<rectangle> dets;
            dets.push_back(openCVRectToDlib(face_rect));
            ///frontal_face_detector ffd = get_frontal_face_detector();
            ///dets = ffd(img);
            cout << "Number of faces detected: " << dets.size() << endl;

            // Now we will go ask the shape_predictor to tell us the pose of
            // each face we detected.
            std::vector<full_object_detection> shapes;

            for (unsigned long j = 0; j < dets.size(); ++j)
            {
                full_object_detection shape = sp(img, dets[j]);
                //dlib::point p = shape.part(0);
                //long l1= p(0);
                cout << "number of parts: "<< shape.num_parts() << endl;


                for(unsigned long i = 0; i <= 67; ++i){
                    cout << (shape.part(i))(0)/float(img_src.cols) << " , " << (shape.part(i))(1)/float(img_src.rows) << endl;
                    landmarks.push_back((shape.part(i))(0)/float(img_src.cols));
                    landmarks.push_back((shape.part(i))(1)/float(img_src.rows));
                }
                // Right eye
                /*
                for(unsigned long i = 43; i <= 47; ++i){
                    cout << (shape.part(i))(0)/float(img_src.cols) << " , " << (shape.part(i))(1)/float(img_src.rows) << endl;
                    landmarks.push_back((shape.part(i))(0)/float(img_src.cols));
                    landmarks.push_back((shape.part(i))(1)/float(img_src.rows));
                }
                // Left eye
                for(unsigned long i = 37; i <= 41; ++i){
                    cout << (shape.part(i))(0)/float(img_src.cols) << " , " << (shape.part(i))(1)/float(img_src.rows) << endl;
                    landmarks.push_back((shape.part(i))(0)/float(img_src.cols));
                    landmarks.push_back((shape.part(i))(1)/float(img_src.rows));
                }*/
                shapes.push_back(shape);
            }

        }
    }
    catch (exception& e)
    {
        cout << "\nexception thrown!" << endl;
        cout << e.what() << endl;
    }
    return landmarks;
}

// ----------------------------------------------------------------------------------------

