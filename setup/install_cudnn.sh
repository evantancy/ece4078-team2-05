#!/bin/bash

file1 = "cudnn-11.0-linux-x64-v8.0.3.33.tgz"
wget https://developer.nvidia.com/compute/machine-learning/cudnn/secure/8.0.3.33/11.0_20200825/cudnn-11.0-linux-x64-v8.0.3.33.tgz
tar -xzvf $file1
sudo cp cuda/include/cudnn*.h /usr/local/cuda/include
sudo cp cuda/lib64/libcudnn* /usr/local/cuda/lib64
sudo chmod a+r /usr/local/cuda/include/cudnn*.h /usr/local/cuda/lib64/libcudnn*


# Install runtime library
file2="libcudnn8_8.0.3.33-1+cuda11.0_amd64.deb"
wget https://developer.nvidia.com/compute/machine-learning/cudnn/secure/8.0.3.33/11.0_20200825/Ubuntu18_04-x64/libcudnn8_8.0.3.33-1%2Bcuda11.0_amd64.deb
sudo dpkg -i $file2

# Install developer library
file3="libcudnn8-dev_8.0.3.33-1+cuda11.0_amd64.deb"
wget https://developer.nvidia.com/compute/machine-learning/cudnn/secure/8.0.3.33/11.0_20200825/Ubuntu18_04-x64/libcudnn8-dev_8.0.3.33-1%2Bcuda11.0_amd64.deb
sudo dpkg -i $file3

# Install code samples
file4="libcudnn8-samples_8.0.3.33-1+cuda11.0_amd64.deb"
wget https://developer.nvidia.com/compute/machine-learning/cudnn/secure/8.0.3.33/11.0_20200825/Ubuntu18_04-x64/libcudnn8-samples_8.0.3.33-1+cuda11.0_amd64.deb
sudo dpkg -i $file4

rm $file1
rm $file2
rm $file3
rm $file4