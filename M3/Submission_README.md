# Milestone 3 (M3)
# Why we chose VGG16
(VGG refers to VGG16)
The first reason we chose VGG is because it is quite similar with Alexnet. Both architechtures has similar working principle, as with most cnn networks (i.e. GoogLeNet,LeNet-5, etc), it applies convolutional layers successively and downsample the spatial domain periodically towards the input. Thus, implementing VGG is very easy (compared to networks such as yolo or SSD due to the different architecture), just change the torchvision models from alexnet to vgg16 and modify the sequential convolutional layers. 
The main reason is because AlexNet uses large and random receptive fields (i.e. 11x11 with a stride of 4 or 5x5 with stride of 1, etc), while VGG uses small and constant sized receptive fields (3x3 with a stride of 1). This means VGG has deeper yet simpler convolutional architechture which results in a better accuracy (we got around 85% accuraccy with Alexnet and above 90% for VGG!). 

# How to test our neural network
- Set `mode` in `master_config.yml` to `master`. This enables the use of `master_config.yml` instead of multiple training parameters
- run nn_test.py to test the trained model
- run nn_detect.py to visualize