#
# Copyright (c) 2019 Intel Corporation.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

Header header

#objects - percepted object category (e.g. "traffic light", "traffic sign", "speed limit")
string[] objects

#poses - poses of percepted objects
geometry_msgs/Pose[] poses

#values - values of percepted object (e.g. "red", "stop", "30")
string[] values

#relative_x_coord - relative x coordinate of within the image (e.g. 0.3 means the object is located at around 30% from left)
float64[] relative_x_coord

#relative_y_coord - relative y coordinate of within the image
float64[] relative_y_coord
