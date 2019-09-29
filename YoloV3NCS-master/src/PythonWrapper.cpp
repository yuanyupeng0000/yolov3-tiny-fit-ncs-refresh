#include <iostream>
#include <boost/python.hpp>
#include <Python.h>

#include "Common.h"
#include "Region.h"

using namespace std;
namespace bp = boost::python;

//#define DEBUG
class YoloDetector
{
private:
	int selection;
	Region region;
public:
	YoloDetector(int _selection)
        {
		selection = _selection;
	}
	virtual ~YoloDetector(){}

	bp::list Detect(bp::object obj, int c, int h, int w,
                       int classes, int imgw, int imgh,
                       float thresh, float nms,
                       int blockwd)
	{
#ifdef DEBUG
		printf("c=%d h=%d w=%d classes=%d imgw=%d imgh=%d thresh=%f nms=%f blockwd=%d\n",
			c,h,w,classes,imgw,imgh,thresh,nms,blockwd);
#endif
		bp::list bplist = bp::list();
		void const *buffer;
		long buflen;
		bool isReadBuffer = !PyObject_AsReadBuffer(obj.ptr(), &buffer, (Py_ssize_t*) &buflen);
		if(!isReadBuffer)
		{
			printf("Cannot read data!\n");
			return bplist;
		}

		float* data = (float*)buffer;                			         
        std::vector<DetectedObject> results;
		region.GetDetections(data, c, h, w, classes, imgw, imgh, thresh, nms, blockwd, results);
		
		for(size_t i = 0; i < results.size(); ++i)
		{
			bplist.append<DetectedObject>(results[i]);
		}

#ifdef DEBUG
		printf("%d\n", results.size());
#endif
		return bplist;
	}

    bp::list TinyV3Detect(bp::object tinyv3_out_1, bp::object tinyv3_out_2, int imgw, int imgh)
    {
        bp::list bplist = bp::list();
        void const *buffer1;
        long buflen1;
        bool isReadBuffer1 = !PyObject_AsReadBuffer(tinyv3_out_1.ptr(), &buffer1, (Py_ssize_t*) &buflen1);
        if(!isReadBuffer1)
        {
            printf("Cannot read data!\n");
            return bplist;
        }

        float* data1 = (float*)buffer1;
                std::vector<DetectedObject> results;

        //region.GetDetections(data, c, h, w, classes, imgw, imgh, thresh, nms, blockwd, results);

        for(size_t i = 0; i < results.size(); ++i)
        {
            bplist.append<DetectedObject>(results[i]);
        }

#ifdef DEBUG
        printf("%d\n", results.size());
#endif
        return bplist;
    }

};

BOOST_PYTHON_MODULE(libpydetector)
{
	bp::class_<YoloDetector>("YoloDetector", bp::init<int>())
        .def("Detect", &YoloDetector::Detect)
        .def("TinyV3Detect", &YoloDetector::TinyV3Detect);

	bp::class_<DetectedObject>("DetectedObject")
		.def_readonly("left", &DetectedObject::left)
		.def_readonly("top", &DetectedObject::top)
		.def_readonly("right", &DetectedObject::right)
		.def_readonly("bottom", &DetectedObject::bottom)
		.def_readonly("confidence", &DetectedObject::confidence)
		.def_readonly("objType", &DetectedObject::objType)
		.def_readonly("name", &DetectedObject::name);
}
