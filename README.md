# YOLOV3 in ROS 
## Detect Objects in images and visualize in RVIZ
Codes are mainly borrowed from [project](https://github.com/vvasilo/yolov3_pytorch_ros). 
It now supports displaying class names of bounding boxes in Chinese. 
To use it, Please:
```
# git clone it 
git clone https://github.com/eriche2016/YOLOV3_In_ROS.git
# place run.sh and src file in catkin_ws folder  
mkdir catkin_ws 
mv YOLOV3_In_ROS/* catkin_ws 
# make it 
catkin_make 
# then run it using run.sh 
```
 
## Install TF for python3 on Ubuntu 16.04
Note: activate openpc_det environment which uses python3 
NOTE: No need to run STEP 0, since I have downloaded the repositories and put it witin this repo.. 
``` 
git clone https://github.com/ros/geometry (branch: noetic-devel) 
git clone https://github.com/ros/geometry2 (branch: noetic-devel)
```
STEP 1: must use c++11 to build it, otherwise cannot pass. 
To avoid adding building option in every file just open: 
```
sudo vim /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake
```
adding the following line: 
```
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
```
after line: 
```
set(CATKIN_TOPLEVEL TRUE)
```
STEP 2: Modify geometry2/tf2/CMakeLists.txt, at the end of the file, replace 
```
catkin_add_gtest(test_transform_datatypes test/test_transform_datatypes.cpp)
target_link_libraries(test_transform_datatypes tf2  ${console_bridge_LIBRARIES})
add_dependencies(test_transform_datatypes ${catkin_EXPORTED_TARGETS})
```
with 
```
catkin_add_gtest(test_transform_datatypes2 test/test_transform_datatypes.cpp)
target_link_libraries(test_transform_datatypes2 tf2  ${console_bridge_LIBRARIES})
add_dependencies(test_transform_datatypes2 ${catkin_EXPORTED_TARGETS})
```
STEP 3: Building for python3, and cmake it 

```
export ROS_PYTHON_VERSION=3
catkin_make
```


