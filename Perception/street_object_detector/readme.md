## About

Uses the semantic segmentation (flat) and rgb cameras to detect street objects (street signs, traffic lights) in the current view (`detect_street_objects()`).
Relevant objects are identified by their color in the semantic segmentation camera, then the resulting pixels mapped to and extracted from the rgb image (`get_color_block()`).
Street signs are then processed by the tesseract OCR library (multiple times, for each defined tesseract configuration), the resulting strings then trimmed, verified and published (`get_ocr_text()`).
Traffic lights are analyzed by the specific dominant color within the extracted image, i.e. if there is more green than yellow and red (for example), by a defined threshold limit, the result green will be published (`detect_traffic_light_color()`).
For further performance improvement, relevant areas to analyze are defined via offsets. For example, street signs are only relevant if located on the right side of the street, whereas traffic lights on the sides are not necessary to analyze, as they are intended for other traffic directions only (this is also done in `detect_street_objects()`).

## Topics

Subscribes to
```
/carla/{}/camera/rgb/front/image_color
/carla/{}/camera/semantic_segmentation/front_flat/image_segmentation
```

Publishes recognized speed limit as Float64 to
```
/psaf/{}/speed_limit
```

Publishes recognized traffic light as String to
```
/psaf/{}/traffic_light
```

Publishes the overall recognition information object as an PerceptionInfo instance to
```
/psaf/{}/perception_info
```

## Installation notes

First, install tesseract OCR via apt by calling `apt-get install tesseract-ocr`. Secondly, install the Python wrapper pytesseract via pip by calling `pip3 install pytesseract`.


## Launch

The street object detector is automatically launched with the ego_vehicle as defined in the
```
roslaunch ego_vehicle ego_vehicle.launch
```
file.
