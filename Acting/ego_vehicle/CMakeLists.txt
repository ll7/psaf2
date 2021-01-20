cmake_minimum_required(VERSION 2.8.3)
project(ego_vehicle)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch)

catkin_python_setup()

roslaunch_add_file_check(launch)

catkin_package(CATKIN_DEPENDS rospy)


install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
