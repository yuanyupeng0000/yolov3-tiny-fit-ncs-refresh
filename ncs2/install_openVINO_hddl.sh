#!/bin/bash
echo "install start"
echo "unpacking l_openvino_toolkit_p_2018.5.445.tgz ... please waite for a moment"
tar -zxf l_openvino_toolkit_p_2018.5.445.tgz
echo "unpacking completed."
cd l_openvino_toolkit_p_2018.5.445
echo "install_cv_sdk_dependencies"
sudo -E ./install_cv_sdk_dependencies.sh
sudo rm -rf /home/$(whoami)/inference_engine_samples
sudo rm -rf /home/$(whoami)/openvino_models

sudo ./install_GUI.sh
sed -i '/source \/opt\/intel\/computer_vision_sdk\/bin\/setupvars.sh/d' ~/.bashrc
sed -i '$a\source \/opt\/intel\/computer_vision_sdk\/bin\/setupvars.sh' ~/.bashrc
source ~/.bashrc
##cd /opt/intel/computer_vision_sdk/deployment_tools/model_optimizer/install_prerequisites
##sudo ./install_prerequisites.sh
cd /opt/intel/computer_vision_sdk/deployment_tools/demo
echo "run basic test demo"
./demo_squeezenet_download_convert_run.sh

echo "------------install opencv-python------------------"
sudo pip3 install opencv-python==3.4.3.18
echo "change cv2 envirement..."
sudo mv /opt/intel/openvino/python/python3.5/cv2.cpython-35m-x86_64-linux-gnu.so /opt/intel/openvino/python/python3.5/cv2.cpython-35m-x86_64-linux-gnu.so.bak
echo "Done!"
