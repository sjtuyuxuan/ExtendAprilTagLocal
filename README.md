# apriltaglocal\_deploy

The goal of this project is to quickly build an accurate (calibration-free) localization system in indoor medium & large-scale scenarios.

In the project, we define the 0th Tag as the base tag for each Tag family. The output pose is related to the base tag. Thus, we use the method of common view to construct a series of April tag pose associations. After that, we will use the available tag in each frame to calculate the global camera pose and finish the localization task.

## Frame work

![ExtendAprilTag 🏖](file:///Users/yuxuan/git/ExtendAprilTagLocal/ExtendAprilTag 🏖.png?msec=1670511561676)

The whole system consists of two parts: the Tag Generator module & the Localization module.

- The Tag Generator module will get tag information (Tag Family, Tag Size / Tag print PPI, Extend Dot size/position) from the YAML file and generate a Tag image with Extend dot.
  
- The localization module will get information (Tag info, Camera Intrinsic parameter/Distortion coefficient, Tag pre-calibration pose) from the YAML file and do the localization task. Finally, the output should be the camera pose related to the base tag.
  

![framework2png](file:///Users/yuxuan/git/ExtendAprilTagLocal/framework_2.png?msec=1670511561686)

## Environment

### Ros Noetic

### Dependency Required

- AprilTag
- Yaml-cpp
- Glog
- Ceres
- Opencv4
- Eigen3

#### Install with `apt-get`

```shell
sudo apt install -y libyaml-cpp-dev libgoogle-glog-dev libeigen3-dev libceres-dev
sudo apt install -y libopencv-dev git cmake
mkdir 3rdparty && cd 3rdparty
git clone https://github.com/AprilRobotics/apriltag && cd apriltag
cmake -B build -DCMAKE_BUILD_TYPE=Release
sudo cmake --build build --target install
```

### Catkin build instruction

```shell
cd ${catkin_ws}/src
git clone https://github.com/sjtuyuxuan/ExtendAprilTagLocal
cd .. && catkin_make
```

##

## How to run

`rosrun extend_april_local localization`

## Sample

![samplegif](file:///Users/yuxuan/git/ExtendAprilTagLocal/sample.gif?msec=1670511561848)

## Test

### Check whether camera is working

   Use rqt to check wether ros camera is working

### Simulate Test with bag

bag is in [link]([bag - Google Drive](https://drive.google.com/drive/folders/1Xrv_PtHaB-Tt48jMuk3dzoAkk-koU0hy))

```shell
rosrun extend_april_local localization
```

```shell
rosbag play test.bag
```

```shell
rviz
```

### Camera calibration

You need to calibrate the camera manually. Ros `camera calibration` maybe a good choice.

You need to get $f_x$ $f_y$ $c_x$ $c_y$ for intrinsic and $k_1$ $k_2$ $p_1$ $p_2$ $(k_3)$ for distortion coefficients. Tipically you should not use camera with high distortion.

### Output check

see whether the tf tree work in rviz
