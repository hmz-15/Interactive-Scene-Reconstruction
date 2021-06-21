# Interactive Scene Reconstruction

### [Project Page](https://sites.google.com/view/icra2021-reconstruction) | [Paper](https://arxiv.org/pdf/2103.16095.pdf) 
<p align="center">
  <img width="500" height="300" src="motivation.jpg">
</p>

This repository contains the implementation of our ICRA2021 paper [Reconstructing Interactive 3D Scenes by Panoptic Mapping and CAD Model Alignments](https://sites.google.com/view/icra2021-reconstruction). 
The proposed pipeline reconstructs an interactive indoor scene from RGBD streams, where objects are replaced by (articulated) CAD models. Represented as a contact graph, 
the reconstructed scene naturally encodes actionable information in terms of environmental kinematics, and can be imported into various simulators to support robot interactions.

The pipeline consists of 3 modules:
- A robust panoptic mapping module that accurately reconstruct the semantics and geometry of objects and layouts, which is a modified version of [Voxblox++](https://github.com/ethz-asl/voxblox-plusplus) but with
improved robustness
- An object-based reasoning module that constructs a contact graph from the dense panoptic map and replaces objects with aligned CAD models
- An interface that converts a contact graph into a kinematic tree in the URDF format, which can be imported into ROS-based simulators

**We will update the code in a few days**

## Todo

- [ ] Upload code for panoptic mapping
- [ ] Upload submodules for panoptic mapping
- [ ] Upload code for CAD replacement
- [ ] Upload code for URDF conversion and scene visualization
- [ ] Upload dataset and use cases
- [ ] Update instructions


