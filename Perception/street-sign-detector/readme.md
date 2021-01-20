## About

Uses semantic segmentation camera and rgb camera to detect new street signs in the current view.
For debuging uncomment line 101 so the detected street signs will get saved in the 'street-sign-detector' folder

## Topics

Subscribes to
```
/carla/{}/semantic_segmentation_front/image
/carla/{}/rgb_front/image
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

The darknet binaries checked in are prebuilt for RTX 2080 GPU and others, otherwise the correct GPU architecture needs to be configured in the `darknet/Makefile` and darknet then rebuilt via `make -j8`. Nvidia CUDA Toolkit and OpenCV 4.5 need to be installed as well, see the `install_cuda.sh` and `install_opencv.sh` files on how to do that.
Furthermore, darknet expects a `yolo-obj_last.weights` file containing the trained network in this package's root - until now, [last year's pretrained file](https://git.rz.uni-augsburg.de/luttkule/carla-praktikum-ws2019/-/blob/master/carla_object_recognition/yolo-obj_last.weights) is used.


## Launch

The street sign detector is automatically launched with the ego_vehicle as of
```
roslaunch ego_vehicle ego_vehicle.launch
```
