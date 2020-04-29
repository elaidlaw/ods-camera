# ods_camera

Clone into ~/catkin_ws/src/

Run the following:
```
cp -r ~/catkin_ws/ods_camera/ods_camera ~/.gazebo/models/
mkdir build
cd build
cmake ..
make
source devel/setup.bash
```
To run the empty world with the camera and ROS node running:
```
roslaunch ods_camera ods_camera.launch
```
The camera will publish /ods_camera/ods_image
