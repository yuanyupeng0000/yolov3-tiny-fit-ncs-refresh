// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#pragma once

#include <string>
#include <vector>
#include <gflags/gflags.h>
#include <iostream>

/// @brief message for help argument
static const char help_message[] = "Print a usage message.";

/// @brief message for images argument
static const char image_message[] = "Required. Path to an image.";

/// @brief message for model argument
static const char model_message[] = "Required. Path to an .xml file with a trained model.";\

/// @brief message for plugin argument
static const char plugin_message[] = "Plugin name. For example MKLDNNPlugin. If this parameter is pointed, " \
"the demo will look for this plugin only";

/// @brief message for assigning cnn calculation to device
static const char target_device_message[] = "Optional. Specify the target device to infer on (the list of available devices is shown below). " \
                                            "Default value is CPU. Use \"-d HETERO:<comma-separated_devices_list>\" format to specify HETERO plugin. " \
                                            "The demo will look for a suitable plugin for the specified device.";

/// @brief message for user library argument
static const char custom_cpu_library_message[] = "Required for CPU custom layers." \
                                                 "Absolute path to a shared library with the kernels implementations.";

/// @brief message for clDNN custom kernels desc
static const char custom_cldnn_message[] = "Required for GPU custom kernels."\
                                            "Absolute path to the xml file with the kernels descriptions.";

/// @brief message for show argument
static const char show_processed_images[] = "Optional. Show processed images. Default value is false.";


/// @brief Define flag for showing help message <br>
DEFINE_bool(h, false, help_message);

/// @brief Define parameter for set image file <br>
/// It is a required parameter
DEFINE_string(i, "", image_message);

/// @brief Define parameter for set model file <br>
/// It is a required parameter
DEFINE_string(m, "", model_message);

/// @brief device the target device to infer on <br>
DEFINE_string(d, "CPU", target_device_message);

/// @brief Absolute path to CPU library with user layers <br>
/// It is a required parameter
DEFINE_string(l, "", custom_cpu_library_message);

/// @brief Define parameter for clDNN custom kernels path <br>
/// Default is ./lib
DEFINE_string(c, "", custom_cldnn_message);

/// @brief Flag to show processed images<br>
/// It is an optional parameter
DEFINE_bool(show, false, show_processed_images);

/**
* @brief This function show a help message
*/
static void showUsage() {
    std::cout << std::endl;
    std::cout << "super_resolution_demo [OPTION]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << std::endl;
    std::cout << "    -h                      " << help_message << std::endl;
    std::cout << "    -i \"<path>\"             " << image_message << std::endl;
    std::cout << "    -m \"<path>\"             " << model_message << std::endl;
    std::cout << "    -d \"<device>\"           " << target_device_message << std::endl;
    std::cout << "    -show                   " << show_processed_images << std::endl;
}
