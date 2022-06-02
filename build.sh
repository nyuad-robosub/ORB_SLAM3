cd Thirdparty
wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/4.4.0.zip
unzip opencv
rm opencv.zip

echo "Configuring and building Thirdparty/DBoW2 ..."

cd ./DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../opencv-4.4.0

echo "Configuring and building Thirdpart/opencv"
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make

cd ../../g2o
echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
