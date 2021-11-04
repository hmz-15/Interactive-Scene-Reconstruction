# Interactive Scene Reconstruction

Given a constructed contact graph with CAD model replacement, we develope an interface to convert a contact graph into a kinematic tree in the URDF format, which can be loaded in ROS environments (r.g., RViz, Gazebo).

**[scene_builder](scene_builder/)** : the ROS package that implements the contact-graph-to-URDF conversion, and recover the articulation of articulated CAD models.

**[gazebo_simulation](gazebo_simulation/gazebo_simulation/)** : the package to generate Gazebo simulation of an interactive scene.

*Note: Before running the following steps, please make sure you have properly install dependencies and build the packages following [this instruction](../assets/INSTALL.md).*

## 1. Prerequisites

### 1.1 CAD dataset

Please make sure you have downloaded and extracted the CAD dataset as described in [CAD replacement](../cad_replacement/). You should have a directory structure as follows:

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

### 1.2 Results of panoptic mapping and cad replacement

Please make sure you have the reconstructed panoptic segments and estimated contact graph under `Interactive-Scene-Reconstruction/output/<scene-folder>/`. This can achieved either by going through the [Robust Panoptic Mapping](../mapping/) and [CAD Replacement](../cad_replacement/), or by downloading an example result from the [Google Drive link](https://drive.google.com/file/d/1P2fgpqfWpkhg-CFKS3YpXGP70aKf9tTe/view?usp=sharing). Please extract the downloaded zip file under `Interactive-Scene-Reconstruction/output/`. The folder structure should look like:

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



where the sub-folder `panoptic_segments/` contains all segmented meshes, and `contact_graph/` contains the contact graph generated from the program. 


## 2. Contact graph to URDF

### 2.1 Xacro scene generation

We provide a scene builder tool to generate xacro interactive scene from the constructed contact graph, segmented meshes, and the CAD database. You can generate the xacro scene by:

```bash
conda activate robot-scene-recon
roslaunch scene_builder generate_xacro_scene.launch scene_name:=sceneNN_test
```

Please refer to the launch file [generate_xacro_scene.launch](launch/generate_xacro_scene.launch) for key arguments to specify the generation process.
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

### 2.2 Scene visualization

We also provide some [launch files](launch/) to visualize the constructed virtual interactive scene. For example, use

```bash
conda deactivate
roslaunch scene_builder view_scene.launch scene:=sceneNN_test
```

to visualize the scene whose name is `sceneNN_test`. 

## 3. Interactive Scene In Gazebo

### 3.1 Gazebo scene generation

To generate the gazebo scene, you need to generate the xacro scene first with `enable_physics` and `enable_gazebo` arguments enabled:

```bash
conda activate robot-scene-recon
roslaunch scene_builder generate_xacro_scene.launch scene_name:=sceneNN_test enable_physics:=true enable_gazebo:=true
```

Then you can generate the gazebo scene via:

```bash
conda deactivate
roslaunch scene_builder generate_gazebo_world.launch scene_name:=sceneNN_test
```

### 3.2 Load gazebo scene

You can load the generated gazebo world into the Gazebo simulator via:
```bash
roslaunch gazebo_simulation gazebo_world.launch scene_name:="sceneNN_test" enable_robot:="false"
```