#!/usr/bin/env python
import os

import rospy

from utils import print_warn, print_info, print_ok, print_err


def sdf_to_gazebo_world(src, dst, world_template):
    template = ""
    with open(world_template, "r") as fin:
        for line in fin:
            template += line
    
    sdf_model = ""
    with open(src, "r") as fin:
        for line in fin:
            if "<sdf version=" in line or "</sdf>" in line:
                continue
            
            if "background/" in line:
                line = line.replace(".obj", ".dae")
            
            sdf_model += line
    
    gazebo_world = template.format(sdf_model)
    with open(dst, "w") as fout:
        fout.write(gazebo_world)
    print(gazebo_world)


def main():
    rospy.init_node("xacro_to_sdf")

    xacro_file_in = rospy.get_param("~xacro_file")
    world_template = rospy.get_param("~world_template")

    urdf_file_out = xacro_file_in[:-5] + "urdf"
    sdf_file_out = xacro_file_in[:-5] + "sdf"
    world_file_out = xacro_file_in[:-5] + "world"

    # convert xacro to urdf
    if 0 != os.system("rosrun xacro xacro --inorder -o {} {}".format(urdf_file_out, xacro_file_in)):
        print_err("[ERROR] Failed to convert from {} to {}".format(xacro_file_in, urdf_file_out))
        exit(0)

    if 0 != os.system("gz sdf -p {} > {}".format(urdf_file_out, sdf_file_out)):
        print_err("[ERROR] Failed to convert from {} to {}".format(urdf_file_out, sdf_file_out))
        exit(0)

    sdf_to_gazebo_world(sdf_file_out, world_file_out, world_template)

    os.system("rm {}".format(urdf_file_out))
    os.system("rm {}".format(sdf_file_out))

    print_ok("[OK] Gazebo world model is saved at: {}".format(world_file_out))
    print_warn("[TIP] Try following command to visualize the gazebo world")

    scene_name = xacro_file_in.split("/")[-2]
    print_info("roslaunch gazebo_simulation gazebo_world.launch scene_name:=\"{}\" enable_robot:=\"false\"".format(scene_name))
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass