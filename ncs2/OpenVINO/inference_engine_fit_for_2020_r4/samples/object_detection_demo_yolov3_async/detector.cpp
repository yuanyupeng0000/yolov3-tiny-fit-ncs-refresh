#include "detector.h"
#include "Common.h"
#include <samples/slog.hpp>
#include <inference_engine.hpp>
#include <iostream>
#include <stdio.h>
#include <gflags/gflags.h>
#include <samples/common.hpp> //#include <samples/ocv_common.hpp>
#include <multi-device/multi_device_config.hpp>
//#include <ext_list.hpp>
#include <chrono>
#include "recognizer.h"
//#define YOLOV3_ORIG
using namespace InferenceEngine;

void Detector::InitIdxSteps(){
    for(int i=0; i<4*24; i++){
       this->idx_steps[i] = 0;
    }
}
void Detector::InitIdxSizeMap(){
    //TODO:change to for loop
    /*this->idx_size_map[0] = this->size_queue0;
    this->idx_size_map[1] = this->size_queue1;
    this->idx_size_map[2] = this->size_queue2;
    this->idx_size_map[3] = this->size_queue3;*/
}

void Detector::InitIdxMatMap(){
    //TODO:change to for loop
    /*this->idx_mat_map[0] = this->mat_queue0;
    this->idx_mat_map[1] = this->mat_queue1;
    this->idx_mat_map[2] = this->mat_queue2;
    this->idx_mat_map[3] = this->mat_queue3;*/
}

int Detector::GetInferIndexes(int index){
    if(this->idx_steps[index] % 2 == 0){
        return 2*index + 1;
    }else{
        return 2*index;
    }
}

Detector::Detector(const std::string& inputXml, const std::string& inputBin, const std::string& inputDevice,
                   const float thresh, const float iou, const int nireq):input_xml(inputXml), thresh(thresh),
    iou(iou), nireq(nireq){
    current_request_id = 0 - nireq;
    opt_config["VPU_HW_STAGES_OPTIMIZATION"] = "NO";
    //printf("current_request_id=%d\n", current_request_id);
    try {
        // --------------------------- 1. Load Plugin for inference engine -------------------------------------
        slog::info << "Loading plugin" << slog::endl;
        ////InferencePlugin plugin = PluginDispatcher({"../../../lib/intel64", ""}).getPluginByDevice(inputDevice);
        //printPluginVersion(plugin, std::cout);

        /**Loading extensions to the plugin **/

        /** Loading default extensions **/
        if (inputDevice.find("CPU") != std::string::npos) {
            /**
             * cpu_extensions library is compiled from the "extension" folder containing
             * custom CPU layer implementations.
            **/
            ////plugin.AddExtension(std::make_shared<Extensions::Cpu::CpuExtensions>());
        }

        /*
        if (!FLAGS_l.empty()) {
            // CPU extensions are loaded as a shared library and passed as a pointer to the base extension
            IExtensionPtr extension_ptr = make_so_pointer<IExtension>(FLAGS_l.c_str());
            plugin.AddExtension(extension_ptr);
        }
        if (!FLAGS_c.empty()) {
            // GPU extensions are loaded from an .xml description and OpenCL kernel files
            plugin.SetConfig({{PluginConfigParams::KEY_CONFIG_FILE, FLAGS_c}});
        }

        // Per-layer metrics
        if (FLAGS_pc) {
            plugin.SetConfig({ { PluginConfigParams::KEY_PERF_COUNT, PluginConfigParams::YES } });
        }*/
        // -----------------------------------------------------------------------------------------------------

        // --------------- 2. Reading the IR generated by the Model Optimizer (.xml and .bin files) ------------
        slog::info << "Loading network files" << slog::endl;
        //CNNNetReader netReader;
        /** Reading network model **/
        ///netReader.ReadNetwork(inputXml);
        //ie.SetConfig(opt_config, std::string("HDDL"));
        cnnNetwork = ie.ReadNetwork(inputXml);
        /** Setting batch size to 1 **/
        slog::info << "Batch size is forced to  1." << slog::endl;
        ///netReader.getNetwork().setBatchSize(1);
        /** Extracting the model name and loading its weights **/
        std::string binFileName = fileNameNoExt(inputXml) + ".bin";
        ///netReader.ReadWeights(binFileName);
        /** Reading labels (if specified) **/
        std::string labelFileName = fileNameNoExt(inputXml) + ".labels";
        std::ifstream inputFile(labelFileName);
        std::copy(std::istream_iterator<std::string>(inputFile),
                  std::istream_iterator<std::string>(),
                  std::back_inserter(labels));
        // -----------------------------------------------------------------------------------------------------

        /** YOLOV3-based network should have one input and three output **/
        // --------------------------- 3. Configuring input and output -----------------------------------------
        // --------------------------------- Preparing input blobs ---------------------------------------------
        slog::info << "Checking that the inputs are as the demo expects" << slog::endl;
        inputInfo = InputsDataMap(cnnNetwork.getInputsInfo());
        if (inputInfo.size() != 1) {
            throw std::logic_error("This demo accepts networks that have only one input");
        }
        InputInfo::Ptr& input = inputInfo.begin()->second;
        auto inputName = inputInfo.begin()->first;
        slog::info << "inputName:" << inputName << slog::endl;
        input->setPrecision(Precision::U8);
        if (/*FLAGS_auto_resize*/false) {
            input->getPreProcess().setResizeAlgorithm(ResizeAlgorithm::RESIZE_BILINEAR);
            input->getInputData()->setLayout(Layout::NHWC);
        } else {
            input->getInputData()->setLayout(Layout::NCHW);
        }
        inputShapes = cnnNetwork.getInputShapes();
        SizeVector& inSizeVector = inputShapes.begin()->second;
        inSizeVector[0] = 1;  // set batch to 1
        cnnNetwork.reshape(inputShapes);
        // --------------------------------- Preparing output blobs -------------------------------------------
        slog::info << "Checking that the outputs are as the demo expects" << slog::endl;
        outputInfo = OutputsDataMap(cnnNetwork.getOutputsInfo());
        /*if (outputInfo.size() != 3) {
            throw std::logic_error("This demo only accepts networks with three layers");
        }*/
        for (auto &output : outputInfo) {
            output.second->setPrecision(Precision::FP32);
            output.second->setLayout(Layout::NCHW);
        }
        // -----------------------------------------------------------------------------------------------------

        //get all avalibale devices
        std::string allDevices = "MULTI:";
        std::vector<std::string> availableDevices = ie.GetAvailableDevices();

        for (auto && device : availableDevices) {
            if(device == "HDDL" || device == "CPU" || device == "GNA"){
                continue;
            }
            allDevices += device;
            allDevices += ((device == availableDevices[availableDevices.size()-1]) ? "" : ",");
        }
        slog::info << "allDevices:" << allDevices << slog::endl;
        // --------------------------- 4. Loading model to the plugin ------------------------------------------
        slog::info << "Loading detector model to the plugin" << slog::endl;
        ExecutableNetwork network;
        if(inputDevice=="MULTI"){
            /*std::string allDevices = "MULTI:";
            std::vector<std::string> myriadDevices = ie.GetMetric("MYRIAD", METRIC_KEY(myriadDevices));
            for (int i = 0; i < myriadDevices.size(); ++i) {
                allDevices += std::string("MYRIAD.")
                                      + myriadDevices[i]
                                      + std::string(i < (myriadDevices.size() -1) ? "," : "");
            }*/
            network = ie.LoadNetwork(cnnNetwork, allDevices, {});
        }
        else{
            network = ie.LoadNetwork(cnnNetwork, inputDevice);
        }
        //ExecutableNetwork network = ie.LoadNetwork(cnnNetwork, inputDevice);

        // -----------------------------------------------------------------------------------------------------

        // --------------------------- 5. Creating infer request -----------------------------------------------
        for(int i=0; i<this->nireq; i++){
            this->IfReqs[i] = network.CreateInferRequestPtr();
        }
        //async_infer_request_next = network.CreateInferRequestPtr();
        //async_infer_request_curr = network.CreateInferRequestPtr();
        // -----------------------------------------------------------------------------------------------------
    }
    catch (const std::exception& error) {
        std::cerr << "[ ERROR ] " << error.what() << std::endl;
    }
    catch (...) {
        std::cerr << "[ ERROR ] Unknown/internal exception happened." << std::endl;
    }
    InitIdxSteps();
    InitIdxSizeMap();
}
/*
void Detector::Detect(const cv::Mat frame){
    try {
        // --------------------------- 6. Doing inference ------------------------------------------------------
        slog::info << "Start inference " << slog::endl;
        bool isAsyncMode = true;
        typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;
        auto total_t0 = std::chrono::high_resolution_clock::now();
        auto wallclock = std::chrono::high_resolution_clock::now();
        double ocv_decode_time = 0, ocv_render_time = 0;
        slog::info << "Start inference " << slog::endl;
        auto t0 = std::chrono::high_resolution_clock::now();
        // Here is the first asynchronous point:
        // in the Async mode, we capture frame to populate the NEXT infer request
        // in the regular mode, we capture frame to the CURRENT infer request
        slog::info << "Start inference " << slog::endl;
        /// auto inputName = inputInfo.begin()->first;
        slog::info << "Start inference " << slog::endl;
        auto t1 = std::chrono::high_resolution_clock::now();
        ocv_decode_time = std::chrono::duration_cast<ms>(t1 - t0).count();

        slog::info << "Start inference " << slog::endl;
        t0 = std::chrono::high_resolution_clock::now();
        // Main sync point:
        // in the true Async mode, we start the NEXT infer request while waiting for the CURRENT to complete
        // in the regular mode, we start the CURRENT request and wait for its completion
        std::cout << "current_request_id " << current_request_id << std::endl;
        ///slog::info << "current_request_id :" << current_request_id << slog::endl;
        if(current_request_id >= 0){
            if (OK == IfReqs[current_request_id]->Wait(IInferRequest::WaitMode::RESULT_READY)) {
                t1 = std::chrono::high_resolution_clock::now();
                ms detection = std::chrono::duration_cast<ms>(t1 - t0);

                t0 = std::chrono::high_resolution_clock::now();
                ms wall = std::chrono::duration_cast<ms>(t0 - wallclock);
                wallclock = t0;

                t0 = std::chrono::high_resolution_clock::now();
                std::ostringstream out;
                out << "OpenCV cap/render time: " << std::fixed << std::setprecision(2)
                    << (ocv_decode_time + ocv_render_time) << " ms";
                cv::putText(frame, out.str(), cv::Point2f(0, 25), cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 255, 0));
                out.str("");
                out << std::fixed << std::setprecision(2) << wall.count() << " ms (" << 1000.f / wall.count() << " fps)";
                cv::putText(frame, out.str(), cv::Point2f(0, 50), cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 0, 255));
                if (!isAsyncMode) {  // In the true async mode, there is no way to measure detection time directly
                    out.str("");
                    out << "Detection time  : " << std::fixed << std::setprecision(2) << detection.count()
                        << " ms ("
                        << 1000.f / detection.count() << " fps";
                    cv::putText(frame, out.str(), cv::Point2f(0, 75), cv::FONT_HERSHEY_TRIPLEX, 0.6,
                                cv::Scalar(255, 0, 0));
                }
                slog::info << 1000.f / detection.count() << " fps\n";
                // ---------------------------Processing output blobs--------------------------------------------------
                // Processing results of the CURRENT request
                unsigned long resized_im_h = inputInfo.begin()->second.get()->getDims()[0];
                unsigned long resized_im_w = inputInfo.begin()->second.get()->getDims()[1];
                const size_t width  = (size_t)frame.size().width;
                const size_t height = (size_t)frame.size().height;
                std::vector<DetectionObject> objects;
                // Parsing outputs
                for (auto &output : outputInfo) {
                    auto output_name = output.first;
                    CNNLayerPtr layer = netReader.getNetwork().getLayerByName(output_name.c_str());
                    Blob::Ptr blob = IfReqs[current_request_id]->GetBlob(output_name);
#ifdef YOLOV3_ORIG
                    ParseYOLOV3Output(layer, blob, resized_im_h, resized_im_w, height, width, thresh, objects);
#else
                    ParseYOLOV3TinyNcsOutput(layer, blob, resized_im_h, resized_im_w, height, width, thresh, objects);
#endif
                }
                slog::info << "current_request_id:" << current_request_id << slog::endl;
                FrameToBlob(frame, IfReqs[current_request_id], "data");
                IfReqs[current_request_id]->StartAsync();
                // Filtering overlapping boxes
                std::sort(objects.begin(), objects.end());
                for (int i = 0; i < objects.size(); ++i) {
                    if (objects[i].confidence == 0)
                        continue;
                    for (int j = i + 1; j < objects.size(); ++j)
                        if (IntersectionOverUnion(objects[i], objects[j]) >= iou)
                            objects[j].confidence = 0;
                }
                // Drawing boxes
                for (auto &object : objects) {
                    if (object.confidence < thresh)
                        continue;
                    auto label = object.class_id;
                    float confidence = object.confidence;
                    if (true) {
                        std::cout << "[" << label << "] element, prob = " << confidence <<
                                  "    (" << object.xmin << "," << object.ymin << ")-(" << object.xmax << "," << object.ymax << ")"
                                  << ((confidence > thresh) ? " WILL BE RENDERED!" : "") << std::endl;
                    }
                    if (confidence > thresh) {
                        // Drawing only objects when >confidence_threshold probability
                        std::ostringstream conf;
                        conf << ":" << std::fixed << std::setprecision(3) << confidence;
                        cv::putText(frame,
                                (label < labels.size() ? labels[label] : std::string("label #") + std::to_string(label))
                                    + conf.str(),
                                    cv::Point2f(object.xmin, object.ymin - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1,
                                    cv::Scalar(0, 0, 255));
                        cv::rectangle(frame, cv::Point2f(object.xmin, object.ymin), cv::Point2f(object.xmax, object.ymax), cv::Scalar(0, 0, 255));
                    }
                }
            //cv::imshow("Detection results", frame);

            t1 = std::chrono::high_resolution_clock::now();
            ocv_render_time = std::chrono::duration_cast<ms>(t1 - t0).count();

            }
        }
        else{
            ///slog::info << "current_request_id:" << current_request_id << slog::endl;
            slog::info << "FrameToBlob" << slog::endl;
            FrameToBlob(frame, IfReqs[current_request_id + nireq], "data");
            slog::info << "StartAsync"<< slog::endl;
            IfReqs[current_request_id + nireq]->StartAsync();
        }

        this->current_request_id += 1;
        if(this->current_request_id >= nireq){
            this->current_request_id = 0;
        }
        auto total_t1 = std::chrono::high_resolution_clock::now();
        ms total = std::chrono::duration_cast<ms>(total_t1 - total_t0);
        std::cout << "Total Inference time: " << total.count() << std::endl;

        //Showing performace results
        if (false) {
            //printPerformanceCounts(*async_infer_request_curr, std::cout);
        }
    }
    catch (const std::exception& error) {
        std::cerr << "[ ERROR ] " << error.what() << std::endl;
        return;
    }
    catch (...) {
        std::cerr << "[ ERROR ] Unknown/internal exception happened." << std::endl;
        return;
    }
    slog::info << "Execution successful" << slog::endl;
    return;
}

int Detector::Detect(const cv::Mat frame, std::vector<DetectionObject>& objects){
    try {
        // --------------------------- 6. Doing inference ------------------------------------------------------
        slog::info << "Start inference " << slog::endl;
        typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;
        // Here is the first asynchronous point:
        // in the Async mode, we capture frame to populate the NEXT infer request
        // in the regular mode, we capture frame to the CURRENT infer request
        auto inputName = inputInfo.begin()->first;
        // Main sync point:
        // in the true Async mode, we start the NEXT infer request while waiting for the CURRENT to complete
        // in the regular mode, we start the CURRENT request and wait for its completion

        if(current_request_id >= 0){
            if (OK == IfReqs[current_request_id]->Wait(IInferRequest::WaitMode::RESULT_READY)) {
                // ---------------------------Processing output blobs--------------------------------------------------
                // Processing results of the CURRENT request
                unsigned long resized_im_h = inputInfo.begin()->second.get()->getDims()[0];
                unsigned long resized_im_w = inputInfo.begin()->second.get()->getDims()[1];
                const size_t width  = (size_t)frame.size().width;
                const size_t height = (size_t)frame.size().height;
                //std::vector<DetectionObject> objects;
                // Parsing outputs
                unsigned long layer_order_id = 0;
                for (auto &output : outputInfo) {
                    auto output_name = output.first;
                    std::cout << "[ INFO ] output_name = " << output_name << std::endl;
                    CNNLayerPtr layer = netReader.getNetwork().getLayerByName(output_name.c_str());
                    Blob::Ptr blob = IfReqs[current_request_id]->GetBlob(output_name);
                    if(this->input_xml.find("SSD") != this->input_xml.npos){
                        std::cout << "[ INFO ] SSD" << std::endl;
                        ParseSSDNcsOutput(layer, blob, resized_im_h, resized_im_w, height, width, thresh, objects);
                    }else if(this->input_xml.find("TinyYoloV3") != this->input_xml.npos){
                        std::cout << "[ INFO ] TinyYoloV3" << std::endl;
                        ParseYOLOV3TinyNcsOutputHW(layer, blob, resized_im_h, resized_im_w, height, width, layer_order_id, thresh, objects);
                        layer_order_id += 1;
                    }else if(this->input_xml.find("YoloV3") != this->input_xml.npos){
                        std::cout << "[ INFO ] YoloV3" << std::endl;
                        ParseYOLOV3Output(layer, blob, resized_im_h, resized_im_w, height, width, thresh, objects);
                    }
                }
                //Showing performace results
                //if (current_request_id >=0) {
                //    printPerformanceCounts(*IfReqs[current_request_id], std::cout);
                //}
                slog::info << "current_request_id:" << current_request_id << slog::endl;
                FrameToBlob(frame, IfReqs[current_request_id], inputName);
                IfReqs[current_request_id]->StartAsync();
                // Filtering overlapping boxes
                std::sort(objects.begin(), objects.end());
                for (int i = 0; i < objects.size(); ++i) {
                    if (objects[i].confidence == 0)
                        continue;
                    for (int j = i + 1; j < objects.size(); ++j)
                        if (IntersectionOverUnion(objects[i], objects[j]) >= iou)
                            objects[j].confidence = 0;
                }
                //for(std::vector<DetectionObject>::iterator iter=objects.begin(); iter!=objects.end(); )
                //{
                //     if(iter->confidence == 0){
                //         iter = objects.erase(iter);
                //     }
                //     else{
                //         iter++;
                //     }
                //}
            }
        }
        else{
            slog::info << "current_request_id:" << current_request_id << slog::endl;
            FrameToBlob(frame, IfReqs[current_request_id + nireq], inputName);
            IfReqs[current_request_id + nireq]->StartAsync();
        }

        this->current_request_id += 1;
        if(this->current_request_id >= nireq){
            this->current_request_id = 0;
        }
    }
    catch (const std::exception& error) {
        std::cerr << "[ ERROR ] " << error.what() << std::endl;
        return -100;
    }
    catch (...) {
        std::cerr << "[ ERROR ] Unknown/internal exception happened." << std::endl;
        return -100;
    }
    slog::info << "Execution successful" << slog::endl;
    return current_request_id;
}
*/
int Detector::Detect(int idx, const cv::Mat frame, std::vector<DetectionObject>& objects){
    try {
        const size_t width  = (size_t)frame.size().width;
        const size_t height = (size_t)frame.size().height;
        struct Image_Size st_image_size;
        st_image_size.h = height;
        st_image_size.w = width;
        //slog::info << "idx_position1 " << idx << slog::endl;
        this->idx_size_map[idx].push(st_image_size);
        this->idx_mat_map[idx].push(frame);
        //slog::info << "idx_position2 " << idx << slog::endl;
        // --------------------------- 6. Doing inference ------------------------------------------------------
        //slog::info << "Start inference " << slog::endl;
        typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;
        // Here is the first asynchronous point:
        // in the Async mode, we capture frame to populate the NEXT infer request
        // in the regular mode, we capture frame to the CURRENT infer request
        auto inputName = inputInfo.begin()->first;
        // Main sync point:
        // in the true Async mode, we start the NEXT infer request while waiting for the CURRENT to complete
        // in the regular mode, we start the CURRENT request and wait for its completion
        this->idx_steps[idx] += 1;
        if(this->idx_steps[idx] == 5){
            this->idx_steps[idx] = 3;
        }
        int temp_request_id = GetInferIndexes(idx);
        std::cout << "[ INFO ] temp_request_id:" << temp_request_id << std::endl;
        if(this->idx_steps[idx] >= 3){
            if (OK == IfReqs[temp_request_id]->Wait(IInferRequest::WaitMode::RESULT_READY)) {
                // ---------------------------Processing output blobs--------------------------------------------------
                // Processing results of the CURRENT request
                const TensorDesc& inputDesc = inputInfo.begin()->second.get()->getTensorDesc();
                unsigned long resized_im_h = getTensorHeight(inputDesc);
                unsigned long resized_im_w = getTensorWidth(inputDesc);

                //const size_t width  = (size_t)frame.size().width;
                //const size_t height = (size_t)frame.size().height;
                struct Image_Size st_image_size = this->idx_size_map[idx].front();
                this->current_result_frame_map[idx] = idx_mat_map[idx].front();
                const size_t width = st_image_size.w;
                const size_t height = st_image_size.h;
                this->idx_size_map[idx].pop();
                this->idx_mat_map[idx].pop();
                //std::vector<DetectionObject> objects;
                // Parsing outputs
                unsigned long layer_order_id = 0;
                for (auto &output : outputInfo) {
                    auto output_name = output.first;
                    CNNLayerPtr layer = cnnNetwork.getLayerByName(output_name.c_str());
                    Blob::Ptr blob = IfReqs[temp_request_id]->GetBlob(output_name);
                    if(this->input_xml.find("SSD") != this->input_xml.npos){
                        //std::cout << "[ INFO ] SSD" << std::endl;
                        ParseSSDNcsOutput(layer, blob, resized_im_h, resized_im_w, height, width, thresh, objects);
                    }else if(this->input_xml.find("TinyYoloV3") != this->input_xml.npos){
                        //std::cout << "[ INFO ] TinyYoloV3" << std::endl;
                        ParseYOLOV3TinyNcsOutput(layer, blob, resized_im_h, resized_im_w, height, width, layer_order_id, thresh, objects);
                        layer_order_id += 1;
                    }else if(this->input_xml.find("YoloV3") != this->input_xml.npos){
                        //std::cout << "[ INFO ] YoloV3" << std::endl;
                        ParseYOLOV3Output(layer, blob, resized_im_h, resized_im_w, height, width, thresh, objects);
                    }else if(this->input_xml.find("yolov5s") != this->input_xml.npos){
                        //std::cout << "[ INFO ] YoloV5" << std::endl;
                        ParseYOLOV5SOutput(layer, blob, resized_im_h, resized_im_w, height, width, layer_order_id, thresh, objects);
                        layer_order_id += 1;
                    }
                }
                // Showing performace results
                //if (current_request_id >=0) {
                //    printPerformanceCounts(*IfReqs[current_request_id], std::cout);
                //}
                //slog::info << "current_request_id:" << temp_request_id << slog::endl;
                FrameToBlob(frame, IfReqs[temp_request_id], inputName);
                IfReqs[temp_request_id]->StartAsync();
                // Filtering overlapping boxes
                std::sort(objects.begin(), objects.end());
                for (int i = 0; i < objects.size(); ++i) {
                    if (objects[i].confidence == 0)
                        continue;
                    for (int j = i + 1; j < objects.size(); ++j)
                        if (IntersectionOverUnion(objects[i], objects[j]) >= iou)
                            objects[j].confidence = 0;
                }
                for(std::vector<DetectionObject>::iterator iter=objects.begin(); iter!=objects.end(); )
                {
                     if(iter->confidence == 0){
                         iter = objects.erase(iter);
                     }
                     else{
                         iter++;
                     }
                }
            }
        }
        else{
            //slog::info << "current_request_id:" << temp_request_id << slog::endl;
            FrameToBlob(frame, IfReqs[temp_request_id], inputName);
            IfReqs[temp_request_id]->StartAsync();
            return -1;
        }
    }
    catch (const std::exception& error) {
        std::cerr << "[ ERROR ] " << error.what() << std::endl;
        return -100;
    }
    catch (...) {
        std::cerr << "[ ERROR ] Unknown/internal exception happened." << std::endl;
        return -100;
    }
    slog::info << "Execution successful" << slog::endl;
    return 0;
}

