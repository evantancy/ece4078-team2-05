#!/bin/bash

sudo apt-get install cmake gcc g++ python3-dev python3-numpy libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libgtk-3-dev libopenexr-dev libwebp-dev libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libatlas-base-dev gfortran

cd ~
# -D PYTHON_DEFAULT_EXECUTABLE=/usr/bin/python3
# -D PYTHON3_NUMPY_INCLUDE_DIRS:PATH=/usr/lib/python3/dist-packages/numpy/core/include
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

cd opencv/
mkdir build
cd build/

nproc
echo "Before proceeding, please check line 36 of this file, for number of threads to use"
echo "This number should be your (nproc-2) or (nproc-4)"
echo "DO NOT USE a shell variable in this script, it will crash"

cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_PYTHON_EXAMPLES=ON \
	-D INSTALL_C_EXAMPLES=OFF \
	-D OPENCV_ENABLE_NONFREE=ON \
	-D WITH_CUDA=ON \
	-D WITH_CUDNN=ON \
	-D OPENCV_DNN_CUDA=ON \
	-D ENABLE_FAST_MATH=1 \
	-D CUDA_FAST_MATH=1 \
	-D WITH_CUBLAS=1 \
	-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
	-D HAVE_opencv_python3=ON \
	-D PYTHON_EXECUTABLE=/usr/bin/python3 \
	-D BUILD_EXAMPLES=ON ..

make -j8
sudo make install
sudo ldconfig

echo "Checking opencv install version ..."
python3 -c "import cv2; print(cv2.__version__)"
