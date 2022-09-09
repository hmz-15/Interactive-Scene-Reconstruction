# Interactive Scene Reconstruction

![ros-version](https://img.shields.io/badge/ubuntu%2016.04+ROS%20kinetic-passing-brightgreen)
![ros-version](https://img.shields.io/badge/ubuntu%2018.04+ROS%20melodic-passing-brightgreen)
![ros-version](https://img.shields.io/badge/ubuntu%2020.04+ROS%20noetic-passing-brightgreen)
![python-version](https://img.shields.io/badge/Python-3.7%2B-blue)

### [Project Page](https://sites.google.com/view/icra2021-reconstruction) | [Paper](https://ieeexplore.ieee.org/document/9561546) | [Arxiv](https://arxiv.org/pdf/2103.16095.pdf)
<p align="center">
  <img width="500" height="300" src="assets/motivation.jpg">
</p>


This repository contains the implementation of our ICRA2021 paper [Reconstructing Interactive 3D Scenes by Panoptic Mapping and CAD Model Alignments](https://sites.google.com/view/icra2021-reconstruction). 
The proposed pipeline reconstructs an interactive indoor scene from RGBD streams, where objects are replaced by (articulated) CAD models. Represented as a contact graph, 
the reconstructed scene naturally encodes actionable information in terms of environmental kinematics, and can be imported into various simulators to support robot interactions.

The pipeline consists of 3 modules:
- A robust panoptic mapping module that accurately reconstruct the semantics and geometry of objects and layouts, which is a modified version of [Voxblox++](https://github.com/ethz-asl/voxblox-plusplus) but with improved robustness. The 2D image segmentation is obtained using [Detectron2](https://github.com/facebookresearch/detectron2)
- An object-based reasoning module that constructs a contact graph from the dense panoptic map and replaces objects with aligned CAD models
- An interface that converts a contact graph into a kinematic tree in the URDF format, which can be imported into ROS-based simulators

## Installation and Usages

- [Installation on Ubuntu](assets/INSTALL.md)
- [Robust Panoptic Mapping](mapping/)
- [Contact Graph and CAD Replacement](cad_replacement/)
- [Interactive Scene Generation and Gazebo Example](interactive_scene/)


## Citing

- Muzhi Han*, Zeyu Zhang*, Ziyuan Jiao, Xu Xie, Yixin Zhu, Song-Chun Zhu, and Hangxin Liu. **Scene Reconstruction with Functional Objects for Robot Autonomy**, International Journal of Computer Vision (IJCV), 2022

- Muzhi Han*, Zeyu Zhang*, Ziyuan Jiao, Xu Xie, Yixin Zhu, Song-Chun Zhu, and Hangxin Liu. **Reconstructing Interactive 3D Scenes by Panoptic Mapping and CAD Model Alignments**, IEEE International Conference on Robotics and Automation (ICRA), 2021

```bibtex
@article{han2022scene,
  title={Scene Reconstruction with Functional Objects for Robot Autonomy},
  author={Han, Muzhi and Zhang, Zeyu and Jiao, Ziyuan and Xie, Xu and Zhu, Yixin and Zhu, Song-Chun and Liu, Hangxin},
  journal={International Journal of Computer Vision (IJCV)},
  year={2022},
  publisher={Springer}
}

@inproceedings{han2021reconstructing,
  title={Reconstructing Interactive 3D Scenes by Panoptic Mapping and CAD Model Alignments},
  author={Han, Muzhi and Zhang, Zeyu and Jiao, Ziyuan and Xie, Xu and Zhu, Yixin and Zhu, Song-Chun and Liu, Hangxin},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  year={2021}
}
```

