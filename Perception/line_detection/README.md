## About
This package detects lines in the semantic-segmentation image. Those lines are then clustered and filtered to compute the distance to the next stopline. 

## Image Processing
The following processing pipeline is used to extract and cluster the lines in the image:

1. Overlay image with a mask, to crop away unnecessary areas (areas of the image where a road cant be)
2. Filter the semantic segmentation image by colors, so that only the roadmarks remain. This yields a binary image of the roadmarks.
3. Apply [Probabilistic Hough Transformation](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html): this yields an array of lines with their lengths and angles. (note that every roadmark-line is represented by a lot of lines after this transformation)
4. Cluster the array of lines according to their angles. We use [Kernel Density Estimation](https://en.wikipedia.org/wiki/Kernel_density_estimation) for this.

Note that there are a number of parameters in every step. We chose them in a way that works well with our semantic-segmentation-camera setup. For other camera setups these parameters might have to be changed. 

The resulting set of line-clusters can be used to calculate an offset from the center of the lane. This was the use-case we originally intended for this package. Another use-case (the one we ended up using it for) is to detect stoplines. But because not every horizontal line on the street is a stopline we have to take the following additional filtering steps: 

* Take the longest line in each cluster
* if 
    * there is a line with an angle smaller than a threshold (```0.05*pi```)
    * and the line is longer than threshold (```0.35*imagewidth```) 
* then take the its negative relative y-position as the relative distance to stop-line. 
* else publish infinity as distance.

## Topics
Subscribes to
```
Topic                                                                      Message Type
/carla/ego_vehicle/camera/semantic_segmentation/front/image_segmentation   sensor_msgs.Image

```

Publishes to
```
Topic                                  Message Type
/psaf/ego_vehicle/roadmark             sensor_msgs.Image
/psaf/ego_vehicle/stopline_distance    std_msgs.Float64

```

## How to launch this node?
```shell
roslaunch line_detection line_detection.launch
```

## Authors
Lukas Hartmann

Valentin Höpfner
