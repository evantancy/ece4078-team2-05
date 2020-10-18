# YOLOv4 in real time
Here we use OpenCV's DNN library
## Setup
1. Download the relevant weights from [here](https://drive.google.com/drive/u/0/folders/1EYMZYM0I26sy8HKrZKJTBQY3htq8cNE4). In our case, use `custom-yolov4-detector_best.weights`
2. Download the configuration file used for training (generating) those weights from [here](https://github.com/evan-tan/image_data/tree/main/cfg). In our case, we are using `custom-yolov4-detector.cfg`
3. Download the file where the labels are defined, i.e. `obj.names` which simply contain the list of classes of our data. Place all these files in the folder called `cfg/`, with the parent folder containing `yolov4.py`

The current implementation uses OpenCV's DNN libraries, you will need to uninstall `opencv-python` and `opencv-contrib-python packages` via `sudo apt remove package_name` or `pip uninstall package_name`
