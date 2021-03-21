cd ~/carla-ros-bridge/psaf2 && 
git submodule update --init &&
cd ~/carla-ros-bridge/catkin_ws/src &&
ln -s ../../psaf2 &&
cd .. && 
catkin_make

# Lanelet
pip3 install opendrive2lanelet && 
sudo pip3 uninstall commonroad-io &&
pip3 install commonroad-io==2020.2

# Behavior Trees
pip3 install py_trees &&
sudo apt install ros-noetic-py-trees-ros

# Tesseract
sudo apt install tesseract-ocr &&
pip3 install pytesseract

#sklearn
pip3 install sklearn

