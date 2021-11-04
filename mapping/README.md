# Robust Panoptic Mapping

The robust panoptic mapping module contains 4 ros packages and 1 python package:

- **panoptic_mapping_pipeline** : provides launch files and options to run the full pipeline

- [**perception_ros**](https://github.com/hmz-15/perception_ros) : image segmentation client and per-frame point cloud segment generation

- [**voxblox-plus-plus**](https://github.com/hmz-15/voxblox-plusplus) : the robust panoptic mapping backend modified on the original [voxblox++](https://github.com/ethz-asl/voxblox-plusplus)

- [**orb_slam_2_ros**](https://github.com/hmz-15/orb_slam_2_ros) : the RGBD SLAM package to compute camera poses when no ground truth camera pose is available, adapted from [orb_slam_2_ros](https://github.com/appliedAI-Initiative/orb_slam_2_ros)

- [**rp_server**](rp_server/) : python3-based image panoptic segmentation server based on [Detectron2](https://github.com/facebookresearch/detectron2)

Please refer to the launch files under `panoptic_mapping_pipeline/launch/` for detailed usages and options.

*Note: Before running the following steps, please make sure you have properly install dependencies and build the packages following [this instruction](../../assets/INSTALL.md).*


## 1. Run Mapping on SceneNN Dataset

[SceneNN dataset](http://103.24.77.34/scenenn/home/) contains RGBD sequences and 3D mesh of indoor scenes. The dataset is available [here](http://103.24.77.34/scenenn/main/). To run panoptic mapping on SceneNN sequences, we first convert the sequences into rosbags using the [scenenn_to_rosbag tool](https://github.com/ethz-asl/scenenn_to_rosbag).

We provide a pre-converted SceneNN rosbag for scene 225, which can be downloaded from this [Google Drive link](https://drive.google.com/file/d/1_4-gcVnPx9S-woaQVBHSTEpW7bX4yRNW/view?usp=sharing).

**Step 1** : Launch the python3-based image segmentation server in one terminal. Please make sure you activate the conda env beforehand. The server is successfully launched when "Server launched at 0.0.0.0:8801" shows up.

``` shell
conda activate robot-scene-recon
cd Interactive-Scene-Reconstruction/mapping/rp_server
python launch_detectron_server.py
```

**Step 2** : Launch the ros nodes in another terminal. By default we use ground truth localization and disable the SLAM module. You may change `sceneNN_test` to any name you like.
``` shell
roslaunch panoptic_mapping_pipeline scenenn_pano_mapping.launch sequence:=sceneNN_test
```

To use estimated camera pose for mapping:
``` shell
roslaunch panoptic_mapping_pipeline scenenn_pano_mapping.launch sequence:=sceneNN_test compute_localization:=true use_GT_pose:=false
```

Please refer to `scenenn_pano_mapping.launch` for more options.


**Step 3** : Finally run the downloaded rosbag. As the whole pipeline runs for ~2fps, we slow down the rosbag play speed.
``` shell
rosbag play -r 0.5 scenenn_225.bag
```

**Step 4** : When the mapping process ends, call the following two ros services to save the mesh of segmented scene and panoptic segments:
``` shell
rosservice call /gsm_node/generate_mesh
rosservice call /gsm_node/extract_instances
```

The output will be saved in the `output` folder under the root directory of `Interactive-Scene-Reconstruction`.


## 2. Run Mapping on Customized Datasets

Running our panoptic mapping module on customized datasets / recorded sequences is almost as simple as writing a new launch file, as long as **rosbags** are available including:

- RGB image topics (set arg `rgb_raw_topic`)
- depth image topics (set arg `depth_raw_topic`)
- camera info topics (set arg `camera_info_topic`)
- (Optional) tf from "world" frame (on the ground) to the camera frame (set arg `rgb_frame_id`)

We provide two example launch files `kinect2_pano_mapping.launch` and `vrep_pano_mapping.launch` that work with rosbags recorded using a hand-held kinect2 camera and captured from the [vrep (CoppeliaSim) simulator](https://www.coppeliarobotics.com/). After setting up everything in the launch file, please just follow the previous section to run the mapping pipeline.


### 2.1 Compute localization with SLAM

When ground truth camera pose is not available (as in most real-world use cases), we use ORB-SLAM2 to localize the camera.

**Specify sequence parameters** : set arguments `dataset` and `ground_axis`; set `compute_localization` as true and `use_GT_pose` as false

**Specify calibrations & orb parameters** : write a new config file `orb_slam2_ros/orb_slam2/$(arg dataset).yaml` following examples `sceneNN.yaml` and `kinect2.yaml`

**Initialize world_to_map transform** : while SLAM provides camera poses w.r.t. the "map" frame, we need to convert the camera pose to the "world" frame (on the ground) so that the reconstruction has the correct orientation and align with the ground. We use two ways to compute the world_to_map transform:

- When ground truth (GT) is available, we use the GT camera pose to initialize the world_to_map transform (set `GT_available` as true)

- When GT is not available, we start sequence with camera in the horizontal position at a certain height above the ground (set `init_height`)

**Save / load map** : while we can directly run ORB-SLAM2 to obtain online camera pose estimation, the reconstructed map will be inconsistent due to the delayed loop closure correction. A better way is to save a map in advance and localize the camera against the map.

- To save the map when the sequence runs to the end :
``` shell
rosservice call /orb_slam2_rgbd/save_map map.bin
```
You can change `map.bin` to any name you want with type `bin`. The map file will be saved under `orb_slam2_ros/orb_slam2/`

- To load the map and localize against the map : set `load_map` as true, and `map_file`. Optionally set `localize_only` as true for reduce computation cost.


### 2.2 Specify panoptic classes
We leverage the panoptic segmentation model in Detectron2 pre-trained on 80 CocoPano classes. Please refer to `panoptic_mapping_pipeline/cfg/pano_class.yaml` for how to specify the panoptic classes accounted during mapping.