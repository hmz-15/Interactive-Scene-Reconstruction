# Interactive Scene Reconstruction

![ros-version](https://img.shields.io/badge/ubuntu%2016.04+ROS%20kinetic-passing-brightgreen)
![ros-version](https://img.shields.io/badge/ubuntu%2018.04+ROS%20melodic-passing-brightgreen)
![ros-version](https://img.shields.io/badge/ubuntu%2020.04+ROS%20noetic-passing-brightgreen)
![python-version](https://img.shields.io/badge/Python-3.7%2B-blue)

<p align="center">
  <img width="500" height="300" src="assets/motivation.jpg">
</p>

This repo implements a machine perception pipeline that reconstructs an interactive indoor scene from RGBD streams, where objects are replaced by (articulated) CAD models. Represented as a contact graph, the reconstructed scene naturally encodes actionable information in terms of environmental kinematics, and can be imported into various simulators (via URDF) to support robot interactions.

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

- Muzhi Han\*, Zeyu Zhang\*, Ziyuan Jiao, Xu Xie, Yixin Zhu, Song-Chun Zhu, and Hangxin Liu. **Scene Reconstruction with Functional Objects for Robot Autonomy**, International Journal of Computer Vision (IJCV), 2022  [[Web](https://sites.google.com/view/ijcv2022-reconstruction)] [[Paper](https://yzhu.io/publication/scenereconstruction2022ijcv/paper.pdf)]

- Muzhi Han\*, Zeyu Zhang\*, Ziyuan Jiao, Xu Xie, Yixin Zhu, Song-Chun Zhu, and Hangxin Liu. **Reconstructing Interactive 3D Scenes by Panoptic Mapping and CAD Model Alignments**, IEEE International Conference on Robotics and Automation (ICRA), 2021 [[Paper](https://yzhu.io/publication/scenereconstruction2021icra/paper.pdf)] [[Arxiv](https://arxiv.org/abs/2103.16095)]

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

## Related Publications in Robot Planning

- Ziyuan Jiao, Yida Niu, Zeyu Zhang, Song-Chun Zhu, Yixin Zhu, and Hangxin Liu. **Sequential Manipulation Planning on Scene Graph**, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2022 [[web](https://sites.google.com/view/planning-on-graph/home)]
- Ziyuan Jiao\*, Zeyu Zhang\*, Weiqi Wang, David Han, Song-Chun Zhu, Yixin Zhu, and Hangxin Liu. **Efficient Task Planning for Mobile Manipulation: a Virtual Kinematic Chain Perspective**, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2021 [[web](https://sites.google.com/view/iros2021-vkc/home/vkc-task?authuser=0)]
- Ziyuan Jiao\*, Zeyu Zhang\*, Xin Jiang, David Han, Song-Chun Zhu, Yixin Zhu, and Hangxin Liu. **Consolidating Kinematic Models to Promote Coordinated Mobile Manipulations**, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2021 [[web](https://sites.google.com/view/iros2021-vkc/home/vkc-motion?authuser=0)]



