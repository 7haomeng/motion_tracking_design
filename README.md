# motion_tracking_design

## Realsense D435
* Open realsense D435 device
```
roslaunch realsense2_camera rs_rgbd.launch
```

## ORB extractor
```
roslaunch ORBextractor ORBextractor.launch 
```

## LK-OpticalFlow
```
roslaunch LKOpticalFlow LK_OpticalFlow.launch
```

## LabelMe
* Follow the step from https://github.com/wkentaro/labelme

## YOLACT
* Follow the step from https://github.com/dbolya/yolact
* Implement the YOLACT with ROS
	Set miniconda environment
	```
	set_conda
	conda activate <env-name> #start up
	conda deactivate <env-name> #close
	```
	Implement the YOLACT with realsense D435
	```
	roslaunch yolact_ros yolact_ros.launch
	```

## ESPNetv2_pytorch

* Set miniconda environment
```
set_conda
conda activate <env-name> #start up
conda deactivate <env-name> #close
```

* Execution ESPNetv2
delete the cached data file (~/ESPNetv2_pytorch/segmentation/city.p)
```
conda activate opencv_build #start up environment
CUDA_VISIBLE_DEVICES=0 python3 main_hao.py --s 2.0 #start to train
python3 prediction_ros.py --s 2.0 --pretrained ./dataSet/hao/espnetv2/results_espnetv2_2.0_2021-12-03_15-31-2.0/model_best.pth

