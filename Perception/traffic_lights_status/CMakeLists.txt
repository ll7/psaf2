cmake_minimum_required(VERSION 2.8.3)
project(traffic_lights_status)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch)

catkin_python_setup()

roslaunch_add_file_check(launch)

catkin_package(CATKIN_DEPENDS rospy)

catkin_install_python(PROGRAMS src/traffic_lights_status/traffic_lights_status.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
