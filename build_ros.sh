echo "Building ROS nodes"
cd ./Examples/ROS/ORB_SLAM3
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release -D'depthai_DIR=~/nr22-software/include/depthai-core/build'
make -j3