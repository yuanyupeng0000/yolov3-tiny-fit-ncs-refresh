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
sudo sed -i '/"total_device_num":            8/s/8/2/g' /opt/intel/computer_vision_sdk_2018.5.445/deployment_tools/inference_engine/external/hddl/config/hddl_autoboot.config
#sudo sed -i '/"total_device_num":            8/s/8/2/g' ${HDDL_INSTALL_DIR}/config/hddl_autoboot.config
echo "Install Completed!"
