# Interactive Scene Reconstruction

### [Project Page](https://sites.google.com/view/icra2021-reconstruction) | [Paper](https://arxiv.org/pdf/2103.16095.pdf) 
<p align="center">
  <img width="500" height="300" src="motivation.jpg">
</p>

This repository contains the implementation of our ICRA2021 paper [Reconstructing Interactive 3D Scenes by Panoptic Mapping and CAD Model Alignments](https://sites.google.com/view/icra2021-reconstruction). 
The proposed pipeline reconstructs an interactive indoor scene from RGBD streams, where objects are replaced by (articulated) CAD models. Represented as a contact graph, 
the reconstructed scene naturally encodes actionable information in terms of environmental kinematics, and can be imported into various simulators to support robot interactions.

The pipeline consists of 3 modules:
- A robust panoptic mapping module that accurately reconstruct the semantics and geometry of objects and layouts, which is a modified version of [Voxblox++](https://github.com/ethz-asl/voxblox-plusplus) but with improved robustness. The 2D image segmentation is obtained using [Detectron2] (https://github.com/facebookresearch/detectron2)
- An object-based reasoning module that constructs a contact graph from the dense panoptic map and replaces objects with aligned CAD models
- An interface that converts a contact graph into a kinematic tree in the URDF format, which can be imported into ROS-based simulators


## Todo

- [ ] Upload code for panoptic mapping
- [ ] Upload submodules for panoptic mapping
- [ ] Upload code for CAD replacement
- [ ] Upload code for URDF conversion and scene visualization
- [ ] Upload dataset and use cases
- [ ] Update instructions

## 1. Installation
### 1.1 Prerequisites
- Ubuntu 16.04 (with ROS Kinetic) or 18.04 (with ROS Melodic)
- Python >= 3.7
- gcc & g++ >= 5.4
- 3 <= OpenCV < 4
- (Optional) Nvidia GPU (with compatible cuda toolkit and cuDNN) if want to run online segmentation

### 1.2 Clone the repository & install catkin dependencies

First create and navigate to your catkin workspace

``` shell
cd <your-working-directory>
mkdir <your-ros-ws>/src && cd <your-ros-ws>
```

Then, initialize the workspace and configure it. (Remember to replace <your-ros-version> by your ros version)

``` shell
catkin init
catkin config --extend /opt/ros/<your-ros-version> --merge-devel 
catkin config --cmake-args -DCMAKE_CXX_STANDARD=14 -DCMAKE_BUILD_TYPE=Release
```
  
Download this repository to your ROS workspace `src/` folder with submodules via:

``` shell
cd src
git clone --recursive https://github.com/hmz-15/Interactive-Scene-Reconstruction.git
```

Then add dependencies specified by .rosinstall using wstool

``` shell
cd Interactive-Scene-Reconstruction
wstool init dependencies
cd dependencies
wstool merge -t . ../mapping/voxblox-plusplus/voxblox-plusplus_https.rosinstall
wstool merge -t . ../mapping/orb_slam2_ros/orb_slam2_ros_https.rosinstall
wstool update
```

### 1.3 Build packages

``` shell
cd <your-ros-ws>
catkin build orb_slam2_ros perception_ros gsm_node -j2
```
  
