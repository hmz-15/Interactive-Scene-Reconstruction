# robot_perception_server

`rp_server` is a python3 package that provides server-based computer vision functions for image semantation based on [Detectron2](https://github.com/facebookresearch/detectron2).

The output format of Detectron2 is available [here](https://detectron2.readthedocs.io/tutorials/models.html#model-output-format). The catagory information is available in COCO dataset.

Please see the `perception_node` in [perception_ros](https://github.com/hmz-15/perception_ros) for an example of the client.