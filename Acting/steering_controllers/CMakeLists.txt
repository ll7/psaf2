cmake_minimum_required(VERSION 2.8.3)
project(steering_controllers)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch)

catkin_python_setup()

roslaunch_add_file_check(launch)

catkin_package(CATKIN_DEPENDS rospy)

catkin_install_python(PROGRAMS src/steering_controllers/pid_control.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
                      
catkin_install_python(PROGRAMS src/steering_controllers/stanley_control.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS src/steering_controllers/vehicle_control.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS src/steering_controllers/__init__.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
