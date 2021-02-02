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
