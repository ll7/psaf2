cmake_minimum_required(VERSION 2.8.3)
project(commonroad_search)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy)
                      
catkin_install_python(PROGRAMS src/commonroad_search/SMP/__init__.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


