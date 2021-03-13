## About

Uses the semantic segmentation and rgb cameras to detect street objects (street signs, traffic lights) in the current view.

## Topics

Subscribes to
```
/carla/{}/camera/rgb/front/image_color
/carla/{}/camera/semantic_segmentation/front/image_segmentation
```

Publishes recognized speed limit as Float64 to
```
/psaf/{}/speed_limit
```

Publishes the overall recognition information object as an PerceptionInfo instance to
```
/psaf/{}/perception_info
```

Speed limit sign and traffic light recognitions are achieved using the semantic segmentation camera.

## Installation notes

First, install tesseract OCR via apt by calling `apt-get install tesseract-ocr`. Secondly, install the Python wrapper pytesseract via pip by calling `pip3 install pytesseract`.


## Launch

The street object detector is automatically launched with the ego_vehicle as of
```
roslaunch ego_vehicle ego_vehicle.launch
```
