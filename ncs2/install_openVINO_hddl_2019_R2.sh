#!/bin/bash
echo "install start"
echo "unpacking l_openvino_toolkit_p_2019.2.242.tgz ... please waite for a moment"
tar -zxf l_openvino_toolkit_p_2019.2.242.tgz
echo "unpacking completed."
cd l_openvino_toolkit_p_2019.2.242
sudo rm -rf /home/$(whoami)/inference_engine_samples
sudo rm -rf /home/$(whoami)/openvino_models

sudo ./install_GUI.sh
cd /opt/intel/openvino/install_dependencies
sudo -E ./install_openvino_dependencies.sh
sed -i '/source \/opt\/intel\/openvino\/bin\/setupvars.sh/d' ~/.bashrc
sed -i '$a\source \/opt\/intel\/openvino\/bin\/setupvars.sh' ~/.bashrc
source ~/.bashrc

echo "install hddl"
source /opt/intel/openvino/bin/setupvars.sh
${HDDL_INSTALL_DIR}/install_IVAD_VPU_dependencies.sh
echo "run basic test demo"
cd /opt/intel/openvino/deployment_tools/demo
./demo_squeezenet_download_convert_run.sh -d HDDL
echo "Done!"

#echo "------------install opencv-python------------------"
#sudo pip3 install opencv-python==3.4.3.18
#echo "change cv2 envirement..."
#sudo mv /opt/intel/openvino/python/python3.5/cv2.cpython-35m-x86_64-linux-gnu.so /opt/intel/openvino/python/python3.5/cv2.cpython-35m-x86_64-linux-gnu.so.bak
