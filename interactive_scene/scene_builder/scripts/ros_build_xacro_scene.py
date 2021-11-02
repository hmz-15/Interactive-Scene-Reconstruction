#!/usr/bin/env python
import os

import rospy

from xacro_scene_builder import XacroSceneBuilder
from scene_builder_config import SceneBuilderConfig
from utils import print_warn, print_info, print_ok, print_err


def build_xacro_scene(config):
    builder = XacroSceneBuilder()
    builder.generate(config)


if __name__ == "__main__":
    rospy.init_node("build_xacro_scene")

    input_scene_dir = rospy.get_param("~input_scene_dir")
    scene_builder_root = rospy.get_param("~scene_builder_root")
    output_dir_name = rospy.get_param("~output_dir_name")
    rigid_mesh_db = rospy.get_param("~rigid_mesh_db")
    articulated_mesh_db = rospy.get_param("~articulated_mesh_db")
    articulated_mesh_default_tf_file = rospy.get_param("~articulated_mesh_default_tf_file")
    enable_vrgym = rospy.get_param("~enable_vrgym")
    enable_physics = rospy.get_param("~enable_physics")
    enable_gazebo = rospy.get_param("~enable_gazebo")

    config = SceneBuilderConfig(
        input_scene_dir=input_scene_dir,
        scene_builder_root=scene_builder_root,
        output_dir_name=output_dir_name,
        rigid_mesh_db=rigid_mesh_db,
        articulated_mesh_db=articulated_mesh_db,
        articulated_mesh_default_tf_file=articulated_mesh_default_tf_file,
        enable_vrgym=enable_vrgym,
        enable_physics=enable_physics,
        enable_gazebo=enable_gazebo
    )

    print(config)
    input("Press <Enter> to continue...")

    build_xacro_scene(config)
