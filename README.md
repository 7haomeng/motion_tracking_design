# motion_tracking_design

## Realsense D435
* Open realsense D435 device
```
roslaunch realsense2_camera rs_rgbd.launch
```

## LK-OpticalFlow
```
roslaunch LKOpticalFlow LK_OpticalFlow.launch
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
python3 prediction_ros.py --s 2.0 --pretrained ./dataSet/hao/espnetv2/results_espnetv2_2.0_2021-09-15_14-47-2.0/model_best.pth

