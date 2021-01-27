## About

Uses the semantic segmentation and rgb cameras to detect street signs in the current view.
For debugging, set `debug = True` in the very beginning of the `street_sign_detector.py` to get more verbose debugging messages and so that detected street signs will be saved as images in the `src/street_sign_detector` folder.

## Topics

Subscribes to
```
/carla/{}/camera/rgb/front/image_color
/carla/{}/camera/rgb/front/camera_info
/carla/{}/camera/semantic_segmentation/front/image_segmentation
/carla/{}/camera/semantic_segmentation/front/camera_info
```

Publishes recognized speed limit as Float64 to
```
/psaf/{}/speed_limit
```

Published the overall recognition information object as an PerceptionInfo instance to
```
/psaf/{}/perception_info
```

Until now, only speed limit sign recognitions are done (depending on the results of the semantic segmentation camera) and published.

## Installation notes

First, the darknet sources need to be cloned from git via `git submodule init --update`.
Afterwards, the correct GPU architecture needs to be configured in the `darknet/Makefile` and darknet then (re)built via `make -j8` run within the darknet repository's folder.
Nvidia CUDA Toolkit and OpenCV 4.5 need to be installed as well, see the `install_cuda.sh` and `install_opencv.sh` files on how to do that.
After the successful execution of `make`, from inside the darknet repository's root folder, run the commands
```
echo export DARKNET_PATH=. >> ~/.bashrc
source ~/.bashrc
```
Basically, the `$DARKNET_PATH` environment variable needs to be pointing to the just newly built library (libdarknet.so).
Furthermore, darknet expects a `yolo-obj_last.weights` file containing the trained network in this package's root - as of now, [last year's pretrained file](https://git.rz.uni-augsburg.de/luttkule/carla-praktikum-ws2019/-/blob/master/carla_object_recognition/yolo-obj_last.weights) is used.


## Launch

The street sign detector is automatically launched with the ego_vehicle as of
```
roslaunch ego_vehicle ego_vehicle.launch
```
