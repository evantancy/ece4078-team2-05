FROM nvidia/cuda:11.4.1-cudnn8-devel-ubuntu18.04

# ARG VARIABLES (only available during build)
ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME="ece4078"
# ENVIRONMENT VARIABLES
ENV HOME "/home/${USER_NAME}"

# setup home directory
WORKDIR /home/
RUN mkdir -p "${HOME}"
WORKDIR ${HOME}

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get autoremove && \
    apt-get clean

RUN apt-get update && apt-get install -y -qq wget \
    unzip \
    curl \
    python3 \
    python3-pip

RUN echo "alias pip3='python3 -m pip'" >> ~/.bash_aliases

# Set pip default to pip3
RUN pip3 install --upgrade --force pip
RUN pip3 install numpy>=1.19.5

RUN apt-get install apt-get update && apt-get install -y -qq python3-tk

# gcc 10
RUN add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt update && \
    apt install gcc-10

# CMake
WORKDIR ${HOME}
RUN apt-get update && apt-get install -y -qq git \
    g++ \
    apt-transport-https \
    ca-certificates \
    gnupg \
    software-properties-common
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
RUN apt-add-repository 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ bionic main'
RUN apt-get update && apt-get install kitware-archive-keyring && \
    rm /etc/apt/trusted.gpg.d/kitware.gpg
RUN apt-get update && apt-get install -y -qq cmake

# OpenCV
ARG OPENCV_VERSION="4.5.1"
ARG CUDA_ARCH="8.6"
WORKDIR ${HOME}
RUN apt-get update && apt-get install -y -qq \
    python3-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
    libopenexr-dev libwebp-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev \
    libgtk-3-dev \
    libatlas-base-dev gfortran

RUN git clone https://github.com/opencv/opencv.git && \
    git clone https://github.com/opencv/opencv_contrib.git
WORKDIR ${HOME}/opencv
RUN git checkout "$OPENCV_VERSION"
WORKDIR ${HOME}/opencv_contrib/
RUN git checkout "$OPENCV_VERSION"

WORKDIR ${HOME}/opencv/
RUN mkdir build/
WORKDIR ${HOME}/opencv/build/
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=$(python3 -c "import sys; print(sys.prefix)") \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D INSTALL_C_EXAMPLES=ON \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D WITH_CUBLAS=1 \
    -D WITH_CUDA=ON \
    -D WITH_CUDNN=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D CUDA_ARCH_BIN="${CUDA_ARCH}" \
    -D OPENCV_EXTRA_MODULES_PATH="${HOME}/opencv_contrib/modules" \
    -D HAVE_opencv_python3=ON \
    -D BUILD_opencv_python3=ON \
    -D PYTHON3_EXECUTABLE="$(which python3)" \
    -D PYTHON_DEFAULT_EXECUTABLE="$(which python3)" \
    -D BUILD_EXAMPLES=ON ..
RUN make --jobs=$(nproc) && \
    make install && \
    ldconfig

# ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && sudo apt update
RUN apt-get install ros-melodic-desktop-full
RUN echo "source /opt/ros/melodic/setup.bash" >> ${HOME}/.bashrc
RUN apt-get install python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential
RUN rosdep init && rosdep update


WORKDIR ${HOME}