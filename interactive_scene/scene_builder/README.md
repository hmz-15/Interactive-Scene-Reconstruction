# SceneBuilder ROS Package

This package provides function that convert a parse graph to URDF scene which could be further loaded into ROS, or some motion planning frameworks, e.g., Tesseract. 

## 1. Dependencies & Installation

Several dependencies need to be installed before using this package:

- **Python packages**: use command `pip install -r requirements.txt` to install these required python dependencies. Noted that is you do not use a python virtual environment, `sudo` is required to install these dependencies.

Once you installed the dependencies, you can install the ROS package via `catkin`.

```bash
catkin build scene_builder
```

In order to use this package, you need to configure your local CAD model database. We provide our database for you, and you can download it here (TBD). You can also use your own CAD database. To directly use the database, you should place the downloaded database at the directory of where the this git repo `<Interactive-Scene-Reconstruction>` placed. You should have a directory structure as follows 

```text
Interactive-Scene-Reconstruction/
    cad_dataset/
        rigid_object/
            xxx.obj
            xxx.mtl
            xxx.png
            ...
        articulated_object/
            fridge_0001/
                xxx.obj
                xxx.mtl
                xxx.urdf
                xxx.xacro
                ...
            fridge_0002/
            ...
        gazebo_world/
            ...
```

## 2. Usage

### 2.1 Input Format

The input should be organized in a folder where all materials are included to construct the URDF scene. The folder should have following structure

```text
<scene-folder>/
	contact_graph/
		contact_graph_cad.json
		...
	panoptic_segments/
		1.ply
		13.ply
		...
```

where the sub-folder `panoptic_segments/` contains all segmented meshes, and `contact_graph/` contains the contact graph generated from the program. Noted that all these files could be generated from the [mapping](../../mapping/) module and the [cad_replacement](../../cad_replacement) module.

## 2.2 Xacro Scene Generation

We provide a scene builder tool to generate xacro interactive scene from the constructed contact graph, segmented meshes, and the CAD database. For your convenience, we provide a launch file for you to use this tool. There are some key arguments to specify the generation process. See more details in file [generate_xacro_scene.launch](launch/generate_xacro_scene.launch).
```xml
<!-- name of the scene/sequence -->
<arg name="scene_name" default="sceneNN_test" />
<!-- directory of the scene -->
<arg name="input_scene_dir" default="$(find scene_builder)/../../output/$(arg scene_name)" />
<!-- the directory of the ROS scene_builder package -->
<arg name="scene_builder_root" default="$(find scene_builder)" />
<!-- the name of the output folder -->
<arg name="output_dir_name" default="$(arg scene_name)" />
<!-- the directory rigid cad databse -->
<arg name="rigid_mesh_db" default="$(find scene_builder)/../../cad_dataset/rigid_object" />
<!-- the directory articulated cad databse -->
<arg name="articulated_mesh_db" default="$(find scene_builder)/../../cad_dataset/articulated_object" />
... and more ...
```

You can generate the xacro scene via the launch file as following

```bash
roslaunch scene_builder generate_xacro_scene.launch scene_name:=sceneNN_test
```

We also provide some [launch files](launch/) to visualize the constructed virtual interactive scene. For example, use

```bash
roslaunch scene_builder view_scene.launch scene:=sceneNN_test
```

to visualize the scene whose name is `scene_001_urdf`. 

*Note: Please build this package before using the launch file to visualize the scene.*

## 2.3 Gazebo Scene Generation

To generate the gazebo scene, you need to generate the xacro scene first, and you need to have the `enable_physics` and `enable_gazebo` parameters enabled as follows,

```xml
<arg name="enable_physics" default="true" />
<arg name="enable_gazebo" default="true" />
```

Once you generate the xacro scene you can generate the gazebo scene based on the xacro scene. We provide a conversion tool for you, and you can use it via the launch file 

For example, you can generate the gazebo world via the launch file, [generate_gazebo_world.launch](launch/generate_gazebo_world.launch), as following 

```bash
roslaunch scene_builder generate_gazebo_world.launch scene_name:=sceneNN_test
```

And you can load the generated gazebo world into the Gazebo simulator via the launch file as following.
```bash
roslaunch gazebo_simulation gazebo_world.launch scene_name:="sceneNN_test" enable_robot:="false"
```