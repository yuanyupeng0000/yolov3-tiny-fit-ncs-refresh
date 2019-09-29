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
echo "Additional Installation Steps for Intel® Vision Accelerator Design with Intel® Movidius™ VPUs"
sudo apt install libusb-1.0-0 libboost-program-options1.58.0 libboost-thread1.58.0 libboost-filesystem1.58.0 libssl1.0.0 libudev1 libjson-c2
sudo usermod -a -G users "$(whoami)"
cd ${HDDL_INSTALL_DIR}
sudo chmod +x ./generate_udev_rules.sh
sudo ./generate_udev_rules.sh /etc/udev/rules.d/98-hddlbsl.rules
sudo sed -i "s/\(.*i2c_i801$\)/#\1/g" /etc/modprobe.d/blacklist.conf
sudo modprobe i2c_i801
kill -9 $(pidof hddldaemon autoboot) >/dev/null 2>&1
echo "install i2c_801 drivers"
cd ${HDDL_INSTALL_DIR}/drivers
sudo chmod +x ./setup.sh
sudo ./setup.sh install
sudo cp -av ${HDDL_INSTALL_DIR}/../97-myriad-usbboot.rules /etc/udev/rules.d/
sudo cp -av ${HDDL_INSTALL_DIR}/etc /
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo ldconfig
sudo apt-get install libboost-python-dev
echo "change config"
sudo sed -i '/"total_device_num":            8/s/8/2/g' ${HDDL_INSTALL_DIR}/config/hddl_autoboot.config
echo "install completed!"
