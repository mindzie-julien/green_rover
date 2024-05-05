#!/bin/bash

sudo apt-get update

sudo apt-get install -y build-essential cmake pkg-config
sudo apt-get install -y libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libv41-dev
sudo apt-get install -y libxvidcore-dev libx264-dev libgtk-3-dev
sudo apt-get install -y libtbb2 libtbb-dev libdc1394-22-dev libv41-dev
sudo apt-get install -y libopenblas-dev libatlas-base-dev libblas-dev
sudo apt-get install -y libjasper-dev liblapack-dev libhdf5-dev
sudo apt-get install -y protobuf-compiler
sudo apt-get install -y libgtk2.0-dev libcanberra-gtk*
sudo apt-get install -y libatlas-base-dev gfortran
sudo apt-get install -y cmake gfortran
#sudo apt-get install -y python2.7-dev python3-dev
pip install numpy
pip3 install numpy

wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.2.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.5.2.zip

unzip opencv.zip
unzip opencv_contrib.zip

cd ~/opencv-4.5.2
mkdir build
cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-4.5.2/modules \
-D BUILD_EXAMPLES=ON ..

cmake -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-4.5.2/modules \

make -j 4

sudo make install
sudo Idconfig

sudo apt-get update
sudo apt-get install imagemagick
