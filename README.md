# hengel_ros
hengel robot autonomous painting - ros package

## Prerequisite:
    $ sudo apt-get install python-setuptools
    $ sudo apt-get install python-skimage




### Requirements:
    $ git clone https://github.com/mariusmuja/flann.git /usr/local/include/flann

    $ cd /usr/local/include/flann
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ sudo make install

If the process is done too fast, erase the directory and repeat again.
Remove /devel and /build and execute catkin_make. (catkin build may not work properly.)