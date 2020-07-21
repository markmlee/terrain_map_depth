# Floor Octomap
This package flattens the Octomap of the floor.


Briefly, the following is happening. A visual SLAM (either [Dense Visual Odometry and SLAM (dvo_slam)](https://github.com/tum-vision/dvo_slam) or [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) runs to localize. Using the pose information and a pointcloud, the [Octomapping Server](https://github.com/OctoMap/octomap_mapping) reconstructs an octomap based on the realworld. As this octomap is noisy, this package tries to remove the noise based on the reconstructed octomap and re-publishes it.

## Installation
We use following non standard ROS-Kinetic packages
```
sudo apt-get install 
    ros-kinetic-octomap 
    ros-kinetic-octomap-mapping
    ros-kinetic-catkin-virtualenv 
```
We recommend to place all packages in the same workspace such as 
```
$ mkdir -p ~/floor_octomap_ws/src
$ cd ~/floor_octomap_ws/src/
$ catkin_init_workspace
```
We assume that [`intel-realsense`](https://github.com/IntelRealSense/realsense-ros)-ROS package is already installed and avaivable in the workspace.
### Installing DVO SLAM
We kindly refer to our other fork of this projcet located at: https://github.com/SuperN1ck/dvo_slam/tree/kinetic-devel

Clone the package into the `src/`-directory of the workspace.
```
$ git clone https://github.com/SuperN1ck/dvo_slam.git
$ cd dvo_slam
$ git checkout kinetic-devel # Not necessary if already on kinetic branch
$ cd ../..
$ catkin_make -j8 -DCMAKE_BUILD_TYPE=Release
```
If `g2o` is not installed please look at the readme in https://github.com/SuperN1ck/dvo_slam/tree/kinetic-devel
### Installing Floor Octomap
Similarly `floor_octomap` can be installed
```
# in 'src/'
$ git clone https://github.com/jyp0802/floor_octomap.git
$ cd ..
$ catkin_make --pkg floor_octomap
```
### Installing ORB SLAM2 (Doesn't work for now. Use DVO SLAM instead)
We use a modified ROS package version of `ORB_SLAM2` which can be found 
```
# in '/src'
$ git clone https://github.com/jyp0802/singlecamera_orbslam2.git
$ cd ..
$ catkin_make --pkg singlecamera_orbslam2
```
### Compiling together
Of course it is also possible to drop the `--pkg`-parameter in order to compile both packages at the same time. It is important though that you make sure to build `dvo_slam` in Release-Mode as otherwise it is really slow.

Source the workspace
```
$ source devel/setup.bash
```

## Running
Once your camera is running you can use our launch-file
```
$ roslaunch floor_octomap modify_octomap_dvo.launch # to use with dvo_slam
OR
$ roslaunch floor_octomap modify_octomap_orb.launch # to use with orb_slam 
```
If using with bagfile
```
$ roslaunch floor_octomap modify_octomap_dvo.launch bagfile:=PATH_TO_BAGFILE 
OR
$ roslaunch floor_octomap modify_octomap_orb.launch bagfile:=PATH_TO_BAGFILE 
```

## Modifications
Changes must be made in the `launch` file and the `flatten.yaml` file.
### launch file
* set the pointcloud to use for octomapping
* set the quaternion of `TF: Initial Camera Position` and `TF: Initial Camera Angle`
* if you want to use pointcloud generated from the depth image, set the depth image on the `Depth Image to Pointcloud`
* if you want to filter the depth image, set the depth image on the `Depth Image Filter`
* `code_backup.launch` (not for running) has brief descriptions of each node/file that is run.
### flatten.yaml file
* `camera_robot_height`: height of the camera from the ground
* `box_XXX`: dimensions for the bounding_box where the octomap will be modified
    * (computation will be slow if set to big)
* you do not need to change the rest
