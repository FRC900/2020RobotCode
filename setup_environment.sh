#!/bin/bash
# Script to setup Jetson TX2 environment. Probably would also work
# with slight modifications on other Jetson hardware

#install basic dependencies
#sudo apt-add-repository ppa:ubuntu-toolchain-r/test -y 
sudo apt update
sudo apt -y upgrade

# These are listed 1 package per line to hopefully make git merging easier
# They're also sorted alphabetically to keep packages from being listed multiple times
sudo apt install -y \
    build-essential \
    can-utils \
    ccache \
    chromium-browser \
    cmake \
    cowsay \
    dbus-x11 \
    exfat-fuse \
    exfat-utils \
    gdb \
    gfortran \
    git \
    gstreamer1.0-plugins-* \
    htop \
    libatlas-base-dev \
    libboost-all-dev \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libclang-9-dev \
    libclang1-9 \
    libeigen3-dev \
    libflann-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libgoogle-perftools-dev \
    libgpiod-dev \
    libgtk2.0-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    libleveldb-dev \
    liblmdb-dev \
    liblua5.3-dev \
    libpcl-dev \
    libproj-dev \
    libsnappy-dev \
    libsuitesparse-dev \
    libtinyxml2-dev \
    net-tools \
    ninja-build \
    nmap \
    ntpdate \
    openssh-client \
    pkg-config \
    pyqt5-dev-tools \
    python-dev \
    python-matplotlib \
    python-numpy \
    python-opencv \
    python-pip \
    python-pyqt5 \
    python-pyqtgraph \
    python-scipy \
	python3 \
    qt4-designer \
    rsync \
    software-properties-common \
    terminator \
    unzip \
    v4l-conf \
    v4l-utils \
    vim-gtk \
    wget \
    xfonts-scalable

#sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 30 --slave /usr/bin/g++ g++ /usr/bin/g++-9
#sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 90 --slave /usr/bin/g++ g++ /usr/bin/g++-7

#install caffe
# cd
# git clone https://github.com/BVLC/caffe.git
# cd caffe
# mkdir build
# cd build

# if [ "$gpu" == "false" ] ; then
    # cmake -DCPU_ONLY=ON ..
# else
    # cmake -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF ..
# fi

# make -j4 all
#make test
#make runtest
# make -j4 install

# Install tinyxml2
cd
git clone https://github.com/leethomason/tinyxml2.git
cd tinyxml2
mkdir build
cd build
cmake -GNinja ..
sudo ninja install
cd ../..
sudo rm -rf tinyxml2

#install zed sdk
wget --no-check-certificate https://download.stereolabs.com/zedsdk/3.2/jp44/jetsons
chmod 755 jetsons
./jetsons
rm ./jetsons

#mount and setup autostart script
sudo mkdir /mnt/900_2
cd ~/2020RobotCode

# Set up can0 network interface
cd
echo "auto can0" > can0
echo "iface can0 inet manual" >> can0
echo "  pre-up /sbin/ip link set can0 type can bitrate 1000000" >> can0
echo "  up /sbin/ifconfig can0 up" >> can0
echo "  down /sbin/ifconfig can0 down" >> can0
sudo mv can0 /etc/network/interfaces.d

sudo bash -c "echo \"# Modules for CAN interface\" >> /etc/modules"
sudo bash -c "echo can >> /etc/modules"
sudo bash -c "echo can_raw >> /etc/modules"
sudo bash -c "echo can_dev >> /etc/modules"
sudo bash -c "echo gs_usb >> /etc/modules"
#sudo bash -c "echo mttcan >> /etc/modules"

# This shouldn't be the least bit dangerous
#sudo rm /etc/modprobe.d/blacklist-mttcan.conf 

# Disable l4tbridge - https://devtalk.nvidia.com/default/topic/1042511/is-it-safe-to-remove-l4tbr0-bridge-network-on-jetson-xavier-/
# sudo rm /etc/systemd/system/nv-l4t-usb-device-mode.sh /etc/systemd/system/multi-user.target.wants/nv-l4t-usb-device-mode.service
sudo systemctl disable nv-l4t-usb-device-mode.service
sudo systemctl stop nv-l4t-usb-device-mode.service

# Set up ssh host config (add port 5801) 
sudo sed "s/#Port 22/Port 22\nPort 5801/g" /etc/ssh/sshd_config > sshd_config && sudo mv sshd_config /etc/ssh

sudo bash -c "echo NTP=10.9.0.2 >> /etc/systemd/timesyncd.conf"
sudo bash -c "echo FallbackNTP=ntp.ubuntu.com >> /etc/systemd/timesyncd.conf"
    
# and keys for connections to Rio
mkdir -p ~/.ssh
cd ~/.ssh
tar -xjf ~/2020RobotCode/jetson_setup/jetson_dot_ssh.tar.bz2 
chmod 640 authorized_keys
cd ~
chmod 700 .ssh

sudo mkdir -p /root/.ssh
sudo tar -xjf /home/ubuntu/2020RobotCode/jetson_setup/jetson_dot_ssh.tar.bz2 -C /root/.ssh
sudo chmod 640 /root/.ssh/authorized_keys
sudo chmod 700 /root/.ssh

cd ~/2020RobotCode
sudo cp ./jetson_setup/10-local.rules ./jetson_setup/99-gpio.rules /etc/udev/rules.d/
sudo service udev reload
sleep 2
sudo service udev restart

if /bin/false; then
    # Kernel module build steps for TX2 : https://gist.github.com/sauhaardac/9d7a82c23e4b283a1e79009903095655
    # Not needed unless Jetpack is updated with a new kernel version and modules
    # for a given kernel version aren't already built
    #
    # This is a PR into the jetsonhacks repo of the same name, will likely be
    # gone the next time anyone looks for it
    # Also, this says Xavier but it really seems to mean Jetpack 4.x, in the
    # case of this particular PR, 4.2. Which is what we're using for the TX2
    # systems as well.
    cd
    git clone https://github.com/klapstoelpiloot/buildLibrealsense2Xavier.git 
    # Note - need to switch back to the default linker to build the kernel image
    cd
    #wget https://developer.nvidia.com/embedded/r32-2-3_Release_v1.0/Sources/T186/public_sources.tbz2 
    wget https://developer.nvidia.com/embedded/dlc/r32-3-1_Release_v1.0/Sources/T186/public_sources.tbz2
    tar -xf public_sources.tbz2 Linux_for_Tegra/source/public/kernel_src.tbz2
    mkdir jetson_kernel
    cd jetson_kernel
    tar -xf ../Linux_for_Tegra/source/public/kernel_src.tbz2
    patch -p0 < ~/2020RobotCode/patch_j120_l4t32.2.3.txt

    ## Apply realsense patches to modules
    cd ~/jetson_kernel/kernel/kernel-4.9
    patch -p1 < ~/buildLibrealsense2Xavier/patches/realsense-camera-formats_ubuntu-bionic-Xavier-4.9.140.patch 
    patch -p1 < ~/buildLibrealsense2Xavier/patches/realsense-metadata-ubuntu-bionic-Xavier-4.9.140.patch
    patch -p1 < ~/buildLibrealsense2Xavier/patches/realsense-hid-ubuntu-bionic-Xavier-4.9.140.patch
    patch -p1 < ~/realsense_src/librealsense-2.31.0/scripts/realsense-powerlinefrequency-control-fix.patch
    # These are for the librealsense code, but don't actually seem to be used
    #patch -p1 < ~/buildLibrealsense2Xavier/patches/model-views.patch
    #patch -p1 < ~/buildLibrealsense2Xavier/patches/incomplete-frame.patch
    rm -rf ~/buildLibrealsense2Xavier

    # turn on various config settings needed for USB tty, realsense, nvme, etc
    zcat /proc/config.gz > .config
    bash scripts/config --file .config \
        --set-str LOCALVERSION -tegra \
        --enable IIO_BUFFER \
        --enable IIO_KFIFO_BUF \
        --module IIO_TRIGGERED_BUFFER \
        --enable IIO_TRIGGER \
        --set-val IIO_CONSUMERS_PER_TRIGGER 2 \
        --module HID_SENSOR_IIO_COMMON \
        --module HID_SENSOR_IIO_TRIGGER \
        --module HID_SENSOR_HUB \
        --module HID_SENSOR_ACCEL_3D \
        --module HID_SENSOR_GYRO_3D \
        --module USB_ACM \
        --module CAN_GS_USB \
        --module JOYSTICK_XPAD \
        --enable CONFIG_BLK_DEV_NVME

    make -j6 clean
    make -j6 prepare
    make -j6 modules_prepare
    make -j6 Image zImage
    make -j6 modules
    sudo make -j6 modules_install
    make -j6 dtbs

    sudo depmod -a

    tar -C ~/jetson_kernel/kernel/kernel-4.9 -cjf ~/j120_hardware_dtb_l4t32-2-3-1.tbz2 \
        `find ~/jetson_kernel -name tegra186-quill-p3310-1000-a00-00-base.dtb | grep -v _ddot_` \
        `find ~/jetson_kernel -name tegra186-quill-p3310-1000-as-0888.dtb | grep -v _ddot_` \
        `find ~/jetson_kernel -name tegra186-quill-p3310-1000-c03-00-base.dtb | grep -v _ddot_` \
        `find ~/jetson_kernel -name tegra186-quill-p3310-1000-c03-00-dsi-hdmi-dp.dtb | grep -v _ddot_` \
        `find ~/jetson_kernel -name tegra186-quill-p3489-0888-a00-00-base.dtb | grep -v _ddot_` \
        `find ~/jetson_kernel -name tegra186-quill-p3489-1000-a00-00-ucm1.dtb | grep -v _ddot_` \
        `find ~/jetson_kernel -name tegra186-quill-p3489-1000-a00-00-ucm2.dtb | grep -v _ddot_` \
        `find ~/jetson_kernel -name tegra194-p2888-0001-p2822-0000.dtb | grep -v _ddot_` \
        `find ~/jetson_kernel -name tegra194-p2888-0001-p2822-0000-maxn.dtb | grep -v _ddot_` \
        `find ~/jetson_kernel -name Image | grep -v _ddot_` \
        `find ~/jetson_kernel -name zImage | grep -v _ddot_` 


    tar -cjf ~/l4t32-2-3-1-modules.tbz2 \
        `find /lib/modules/4.9.140-tegra/kernel -name hid-sensor-iio-common.ko` \
        `find /lib/modules/4.9.140-tegra/kernel -name hid-sensor-trigger.ko` \
        `find /lib/modules/4.9.140-tegra/kernel -name hid-sensor-hub.ko` \
        `find /lib/modules/4.9.140-tegra/kernel -name hid-sensor-accel-3d.ko` \
        `find /lib/modules/4.9.140-tegra/kernel -name hid-sensor-gyro-3d.ko` \
        `find /lib/modules/4.9.140-tegra/kernel -name cdc-acm.ko` \
        `find /lib/modules/4.9.140-tegra/kernel -name gs_usb.ko` \
        `find /lib/modules/4.9.140-tegra/kernel -name xpad.ko`

    # make -j6 M=drivers/usb/class
    # make -j6 M=drivers/usb/serial
    # make -j6 M=drivers/net/can
    # make -j6 M=net/can
    # sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/serial
    # sudo cp drivers/usb/class/cp210x-acm.ko /lib/modules/`uname -r`/kernel/drivers/usb/serial/cp210x-acm.ko
    # sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/class
    # sudo cp drivers/usb/serial/cdc-acm.ko /lib/modules/`uname -r`/kernel/drivers/usb/class/cdc-acm.ko
    # sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/class
    # sudo cp drivers/net/can/usb/gs_usb.ko /lib/modules/`uname -r`/kernel/drivers/net/can/usb

    # sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/joystick
    # sudo cp xpad.ko /lib/modules/`uname -r`/kernel/drivers/joystick/xpad.ko
    # sudo depmod -a
fi

# Clean up Jetson
sudo rm -rf /home/nvidia/cudnn /home/nvidia/OpenCV /home/nvidia/TensorRT /home/nvidia/libvisionworkd*
# Save ~400MB
sudo apt remove --purge -y thunderbird libreoffice-*
# Disable automatic updates
sudo sed -i -e 's/APT::Periodic::Update-Package-Lists "1"/APT::Periodic::Update-Package-Lists "0"/' /etc/apt/apt.conf.d/10periodic

# Install CTRE & navX libs
mkdir -p /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include 
mkdir -p /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/ctre 
cd /home/ubuntu
wget -e robots=off -U mozilla -r -np http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/ -A "*5.18.3*,firmware-sim*zip" -R "md5,sha1,pom,jar,*windows*,*debug*"
cd /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include 
find /home/ubuntu/devsite.ctr-electronics.com -name \*headers\*zip | grep -v debug | xargs -n 1 unzip -o 
cd /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/ctre 
find /home/ubuntu/devsite.ctr-electronics.com -name \*linux\*zip | grep -v debug | xargs -n 1 unzip -o 
rm -rf /home/ubuntu/devsite.ctr-electronics.com 

cd /home/ubuntu 
wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.400/navx-cpp-3.1.400-headers.zip 
mkdir -p /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include/navx 
cd /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include/navx 
unzip -o /home/ubuntu/navx-cpp-3.1.400-headers.zip 
rm /home/ubuntu/navx-cpp-3.1.400-headers.zip 
cd /home/ubuntu 
wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.400/navx-cpp-3.1.400-linuxathena.zip 
mkdir -p /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/navx 
cd /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/navx 
unzip -o /home/ubuntu/navx-cpp-3.1.400-linuxathena.zip 
rm /home/ubuntu/navx-cpp-3.1.400-linuxathena.zip 
cd /home/ubuntu 
wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.400/navx-cpp-3.1.400-linuxathenastatic.zip 
cd /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/navx 
unzip -o /home/ubuntu/navx-cpp-3.1.400-linuxathenastatic.zip 
rm /home/ubuntu/navx-cpp-3.1.400-linuxathenastatic.zip 

# And Rev sparkmax stuff
cd /home/ubuntu
wget http://www.revrobotics.com/content/sw/max/sdk/SPARK-MAX-SDK-v1.5.2.zip
mkdir sparkmax
cd sparkmax
unzip ../SPARK-MAX-SDK-v1.5.2.zip
rm ../SPARK-MAX-SDK-v1.5.2.zip
mkdir -p /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/rev
cd /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/rev
find /home/ubuntu/sparkmax/maven/com/revrobotics/frc/SparkMax-cpp -name \*athena\*zip | grep -v debug | xargs -n 1 unzip -o
find /home/ubuntu/sparkmax/maven/com/revrobotics/frc/SparkMax-cpp -name \*linux\*zip | grep -v debug | xargs -n 1 unzip -o
find /home/ubuntu/sparkmax/maven/com/revrobotics/frc/SparkMax-driver -name \*athena\*zip | grep -v debug | xargs -n 1 unzip -o
find /home/ubuntu/sparkmax/maven/com/revrobotics/frc/SparkMax-driver -name \*linux\*zip | grep -v debug | xargs -n 1 unzip -o
cd /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include
find /home/ubuntu/sparkmax/maven/com/revrobotics/frc/SparkMax-cpp -name \*header\*zip | grep -v debug | xargs -n 1 unzip -o
find /home/ubuntu/sparkmax/maven/com/revrobotics/frc/SparkMax-driver -name \*header\*zip | grep -v debug | xargs -n 1 unzip -o
rm -rf /home/ubuntu/sparkmax

# Install wpilib headers by copying them from the local maven dir
# TODO - need to update to acutal 2020 filenames once they are released
cd /home/ubuntu 
wget https://github.com/wpilibsuite/allwpilib/releases/download/v2020.3.2/WPILib_Linux-2020.3.2.tar.gz 
mkdir -p /home/ubuntu/wpilib/2020 
cd /home/ubuntu/wpilib/2020 
tar -xzf /home/ubuntu/WPILib_Linux-2020.3.2.tar.gz 
rm /home/ubuntu/WPILib_Linux-2020.3.2.tar.gz 
cd /home/ubuntu/wpilib/2020/tools 
python3 ToolsUpdater.py 
mkdir -p /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/wpilib 
cd /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/wpilib 
find ../../../.. -name \*athena\*zip | xargs -n1 unzip -o 
mkdir -p /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include/wpilib 
cd /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include/wpilib 
find ../../../.. -name \*headers\*zip | xargs -n1 unzip -o 
rm -rf /home/ubuntu/wpilib/2020/maven /home/ubuntu/wpilib/2020/jdk
sed -i -e 's/   || defined(__thumb__) \\/   || defined(__thumb__) \\\n   || defined(__aarch64__) \\/' /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include/wpilib/FRC_FPGA_ChipObject/fpgainterfacecapi/NiFpga.h

# Set up prereqs for deploy script
mv ~/2020RobotCode ~/2020RobotCode.orig
ln -s ~/2020RobotCode.orig ~/2020RobotCode
mkdir -p ~/2020RobotCode.prod/zebROS_ws
mkdir -p ~/2020RobotCode.dev/zebROS_ws

sudo mkdir -p /usr/local/zed/settings
sudo chmod 755 /usr/local/zed/settings
sudo cp ~/2020RobotCode/calibration_files/*.conf /usr/local/zed/settings
sudo chmod 644 /usr/local/zed/settings/*

cp ~/2020RobotCode/.vimrc ~/2020RobotCode/.gvimrc ~
sudo cp ~/2020RobotCode/kjaget.vim /usr/share/vim/vim80/colors

git config --global user.email "progammers@team900.org"
git config --global user.name "Team900 Jetson TX2"

# Set up Gold linker - speed up libPCL links
sudo update-alternatives --install "/usr/bin/ld" "ld" "/usr/bin/ld.gold" 20
sudo update-alternatives --install "/usr/bin/ld" "ld" "/usr/bin/ld.bfd" 10

echo | sudo update-alternatives --config ld

sudo ccache -C
sudo ccache -c
sudo rm -rf /home/ubuntu/.cache /home/ubuntu/.ccache

sudo ln -s /usr/include/opencv4 /usr/include/opencv

echo "source /home/ubuntu/2020RobotCode/zebROS_ws/command_aliases.sh" >> /home/ubuntu/.bashrc

# Install make 4.3 (>4.2 is required for -flto=jobserver support
cd
wget https://ftp.gnu.org/gnu/make/make-4.3.tar.gz
tar -xf make-4.3.tar.gz
mkdir make-4.3/build
cd make-4.3/build
../configure --prefix=/usr
sudo make -j`nproc --all` install
cd
rm -rf make-4.3*


# Give the ubuntu user dialout permission, which is used by the ADI IMU 
sudo adduser ubuntu dialout

git clone https://github.com/VundleVim/Vundle.vim.git /home/ubuntu/.vim/bundle/Vundle.vim
vim +PluginInstall +qall
ln -sf /home/ubuntu/.vim/bundle/vim-ros-ycm/.ycm_extra_conf.py /home/ubuntu/.vim/bundle/vim-ros-ycm/ycm_extra_conf.py
cd /home/ubuntu/.vim/bundle/YouCompleteMe
git fetch origin
git submodule update --init --recursive
python3 ./install.py --clang-completer --system-libclang --ninja 

# Install tensorflow on Jetson
#sudo apt update
#sudo apt install libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran python3-pip python3-opencv python3-pil python3-matplotlib
#sudo pip3 install --install-option="--jobs=6" -U pip testresources setuptools
#sudo pip3 install --install-option="--jobs=6" -U numpy==1.16.1 future==0.17.1 mock==3.0.5 h5py==2.9.0 keras_preprocessing==1.0.5 keras_applications==1.0.8 gast==0.2.2 futures pybind11
#sudo pip3 install --install-option="--jobs=6" --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v44 tensorflow==1.15.2+nv20.04

#Build working version of protobuf from source
mkdir -p ~/src
cd ~/src
sudo apt-get install -y autoconf libtool
if [ ! -f protobuf-python-3.12.3.zip ]; then
  wget https://github.com/protocolbuffers/protobuf/releases/download/v3.12.3/protobuf-all-3.12.3.zip
fi
if [ ! -f protoc-3.12.3-linux-aarch_64.zip ]; then
  wget https://github.com/protocolbuffers/protobuf/releases/download/v3.12.3/protoc-3.12.3-linux-aarch_64.zip
fi

echo "** Install protoc"
unzip protobuf-all-3.12.3.zip
unzip protoc-3.12.3-linux-aarch_64.zip -d protoc-3.12.3
sudo cp protoc-3.12.3/bin/protoc /usr/local/bin/protoc
echo "** Build and install protobuf-3.12.3 libraries"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp
cd protobuf-3.12.3/
cd cmake
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -Dprotobuf_BUILD_TESTS=OFF ..

#./autogen.sh
#./configure --prefix=/usr/local

sed -i 's/-g -O2/-g -O2 -fPIC/' Makefile
make -j`nproc --all`
#make -j`nproc --all` check
sudo make -j`nproc --all` install
sudo ldconfig
cd ../..

# Fix bug in released version of catkin_tools
sudo sed -i 's/ errno.EINTR/ errno.EINTR and e.errno != errno.EAGAIN/'  /usr/lib/python2.7/dist-packages/catkin_tools/execution/job_server.py

echo "** Update python protobuf module"
# remove previous installation of python protobuf module
sudo python -m pip uninstall -y protobuf
sudo python -m pip install Cython
cd python/
# force compilation with c++11 standard
python setup.py build --cpp_implementation
# Probably fails?
python setup.py test --cpp_implementation
sudo python setup.py install --cpp_implementation

cd &&\
    wget https://github.com/git-lfs/git-lfs/releases/download/v2.11.0/git-lfs-linux-arm64-v2.11.0.tar.gz &&\
	mkdir git-lfs-install &&\
	cd git-lfs-install &&\
	tar -xzf ../git-lfs-linux-arm64-v2.11.0.tar.gz &&\
	sudo ./install.sh &&\
	cd &&\
	rm -rf git-lfs-linux-amd64-v2.11.0.tar.gz git-lfs-install &&\
	git lfs install &&\
	cd ~/2020RobotCode &&\
	git lfs pull

cd ~/2020RobotCode
sudo apt-get install -y libhdf5-serial-dev hdf5-tools
sudo dpkg -i libnccl*arm64.deb
sudo python -m pip install -U pip six numpy wheel setuptools mock h5py
sudo python -m pip install -U keras_applications
sudo python -m pip install -U keras_preprocessing
sudo python -m pip install tensorflow-1.15.3-*.whl

# Patch catkin tools/pkg for faster builds
cd /usr/lib/python2.7/dist-packages
sudo patch -p0 < ~/2020RobotCode/catkin_pkg.patch
sudo patch -p0 < ~/2020RobotCode/catkin_tools.patch

cd /home/ubuntu
git clone https://github.com/tensorflow/models.git
cd models
git submodule init
git submodule update
cd /home/ubuntu/models/research
protoc object_detection/protos/*.proto --python_out=.
sudo python -m pip install --no-cache-dir .
cd slim
sudo python -m pip install --no-cache-dir .
