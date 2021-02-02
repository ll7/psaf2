#!/bin/sh

wget https://github.com/opencv/opencv/archive/4.5.1.zip
unzip 4.5.1.zip

cd 4.5.1
mkdir -p build
cd build

cmake ../opencv-4.5.1/

make -j8 && sudo make install

sudo find / -name "libopencv_highgui.so.4.5"
