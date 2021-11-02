import os
from utils import load_json
from utils import print_err, print_warn


#################################################################
# Deprecated: replace meshlabserver with pymeshlab
#################################################################
# # get MESH_LAB_SERVER executable
# if os.system("type meshlab.meshlabserver") == 0:
#     # meshlab is installed via snap
#     MESH_LAB_SERVER = "meshlab.meshlabserver"
# elif os.system("type meshlabserver") == 0:
#     # meshlab is installed via apt
#     MESH_LAB_SERVER = "meshlabserver"
# else:
#     print_warn("[WARN] Fail to find meshlabserver, use PyMeshLab instead")
#     MESH_LAB_SERVER = "pymeshlab"
    
# MESHLAB_COLOR_TO_TEXTURE_TEMPLATE = "assets/meshlab_scripts/vertex_color_to_texture.mlx"
# TMP_MESHLAB_SCRIPT = ".tmp_meshlab_filter_script.mlx"
# MESHLAB_EXE_TEMPLATE_SCRIPT = MESH_LAB_SERVER + " -i {} -o {} -m {} -s {}"
# MESHLAB_EXE_TEMPLATE = MESH_LAB_SERVER + " -i {} -o {} -m vc vn fc wt"


VERTEX_TO_FACE_TEXTURE_DIM = 4096
MESHLAB_TEXTURE_FILE_TEMPLATE = "{}_texture.png"


SCENE_PG_FILENAME = "contact_graph/contact_graph_cad.json"
SCENE_SEGMENTS_DIR = "panoptic_segments"

MAIN_XACRO_FILENAME = "main.xacro"
RIGIT_OBJ_XACRO_FILENAME = "rigid_objects.xacro"
LINK_TO_MESH_FILENAME = "link_to_mesh.csv"
PARENT_CHILD_PAIR_FILENAME = "link_parent_child_pair.csv"

DEFAULT_SCENE_ROOT = "Room_0_link"
RIGID_OBJ_XACRO_MACRO_NAME = "create_rigid_objects"

XACRO_INC_SCENE_PREFIX = "$(find scene_builder)"
XACRO_PKG_SCENE_PREFIX = "package://scene_builder"

RIGID_DEFAULT_XYZ = [0, 0, 0]
RIGID_DEFAULT_RPY = [0, 0, 0]
RIGID_DEFAULT_SCALE = [1, 1, 1]
RIGID_DEFAULT_MASS = 5.0
RIGID_DEFAULT_INERTIA = [1.0, 0.0, 0.0, 1.0, 0.0, 1.0]
RIGID_DEFAULT_JOINT_TYPE = "fixed"

SCENE_BUILDER_OUTPUT_DIR = "output"

RIGID_MESH_FOLDER = "rigid"
INTERACTIVE_MESH_FOLDER = "interactive"
BACKGROUND_MESH_FOLDER = "background"

SCENE_RIGID_MESH_FOLDER = "assets/{}".format(RIGID_MESH_FOLDER)
SCENE_INTERACTIVE_MESH_FOLDER = "assets/{}".format(INTERACTIVE_MESH_FOLDER)
SCENE_BACKGROUND_MESH_FOLDER = "assets/{}".format(BACKGROUND_MESH_FOLDER)

DATABASE_RIGID_MESH_DIR = "assets/{}".format("rigid_open3d")
DATABASE_INTERACTIVE_MESH_DIR = "assets/{}".format(INTERACTIVE_MESH_FOLDER)
DATABASE_INTERACTIVE_DEFAULT_TF_FILE = "assets/{}/interactive_transform.json".format(INTERACTIVE_MESH_FOLDER)

VRGYM_SCALED_MESH_DIR_ROOT = "assets/scaled"


INTERACTIVE_CATEGORY = [
    "Cabinet",
    "Fridge",
    "Microwave",
    "Drawer",
    "Door",
    "Refrigerator"
]

BACKGROUND_CATEGORY = [
    "Ceiling",
    "Floor",
    "Wall",
    "Background"
]


INTERACTIVE_LINK_TO_MESH = {
    "cabinet_0001": {
        "": "none_motion.obj",
        "_dof_rootd_Bb001_r": "dof_rootd_Bb001_r.obj"
    },
    "cabinet_0007": {
        "": "none_motion.obj",
        "_dof_rootd_Aa002_t": "dof_rootd_Aa002_t.obj",
        "_dof_rootd_Aa003_t": "dof_rootd_Aa003_t.obj",
        "_dof_rootd_Aa004_t": "dof_rootd_Aa004_t.obj",
        "_dof_rootd_Aa005_t": "dof_rootd_Aa005_t.obj",
        "_dof_rootd_Aa001_t": "dof_rootd_Aa001_t.obj"
    },
    "fridge_0001": {
        "": "none_motion.obj",
        "_dof_rootd_Aa001_r": "dof_rootd_Aa001_r.obj",
        "_dof_rootd_Aa002_r": "dof_rootd_Aa002_r.obj",
        "_dof_rootd_Ba001_t": "dof_rootd_Ba001_t.obj",
        "_dof_rootd_Ba002_t": "dof_rootd_Ba002_t.obj",
        "_dof_rootd_Ba003_t": "dof_rootd_Ba003_t.obj"
    },
    "fridge_0002": {
        "": "none_motion.obj",
        "_dof_rootd_Aa002_r": "dof_rootd_Aa002_r.obj",
        "_dof_rootd_Ba003_t": "dof_rootd_Ba003_t.obj"
    },
    "fridge_0003": {
        "": "none_motion.obj",
        "_dof_rootd_Aa002_r": "dof_rootd_Aa002_r.obj"
    },
    "iai_fridge": {
        "": "Fridge.dae",
        "_door": "FridgeDoor.dae",
        "_door_handle": "VHandle90.dae"
    },
    "microwave_0001": {
        "": "none_motion.obj",
        "_dof_rootd_Aa001_r": "dof_rootd_Aa001_r.obj"
    },
    "fixed_13_cabinet": {
        "": "free_13_cabinet_floor_none_motion_z_up.obj",
        "_free_13_cabinet_floor_dof_rootd_Aa001_t_z_up": "free_13_cabinet_floor_dof_rootd_Aa001_t_z_up.obj",
        "_free_13_cabinet_floor_dof_rootd_Aa002_t_z_up": "free_13_cabinet_floor_dof_rootd_Aa002_t_z_up.obj",
        "_free_13_cabinet_floor_dof_rootd_Aa003_t_z_up": "free_13_cabinet_floor_dof_rootd_Aa003_t_z_up.obj",
        "_free_13_cabinet_floor_dof_rootd_Aa004_t_z_up": "free_13_cabinet_floor_dof_rootd_Aa004_t_z_up.obj",
        "_free_13_cabinet_floor_dof_rootd_Aa005_t_z_up": "free_13_cabinet_floor_dof_rootd_Aa005_t_z_up.obj"
    }
}


# The placeholder tag for the parent link of the baselink of
# an interactive object
INTERACTIVE_BASE_PARENT_TAG = "#BASE_PARENT_TAG#"

# (parent_link_name, child_link_name) relation
# The actual link_name is prefix + link_name
INTERACTIVE_LINK_PAIR = {
    "cabinet_0001": [
        [INTERACTIVE_BASE_PARENT_TAG, ""],
        ["", "_dof_rootd_Bb001_r"]
    ],
    "cabinet_0007": [
        [INTERACTIVE_BASE_PARENT_TAG, ""],
        ["", "_dof_rootd_Aa002_t"],
        ["", "_dof_rootd_Aa003_t"],
        ["", "_dof_rootd_Aa004_t"],
        ["", "_dof_rootd_Aa005_t"],
        ["", "_dof_rootd_Aa001_t"]
    ],
    "fridge_0001": [
        [INTERACTIVE_BASE_PARENT_TAG, ""],
        ["", "_dof_rootd_Aa001_r"],
        ["", "_dof_rootd_Aa002_r"],
        ["", "_dof_rootd_Ba001_t"],
        ["", "_dof_rootd_Ba002_t"],
        ["", "_dof_rootd_Ba003_t"]
    ],
    "fridge_0002": [
        [INTERACTIVE_BASE_PARENT_TAG, ""],
        ["", "_dof_rootd_Aa002_r"],
        ["", "_dof_rootd_Ba003_t"]
    ],
    "fridge_0003": [
        [INTERACTIVE_BASE_PARENT_TAG, ""],
        ["", "_dof_rootd_Aa002_r"]
    ],
    "iai_fridge": [
        [INTERACTIVE_BASE_PARENT_TAG, ""],
        ["", "_door"],
        ["_door", "_door_handle"]
    ],
    "microwave_0001": [
        [INTERACTIVE_BASE_PARENT_TAG, ""],
        ["", "_dof_rootd_Aa001_r"]
    ],
    "fixed_13_cabinet": [
        [INTERACTIVE_BASE_PARENT_TAG, ""],
        ["", "_free_13_cabinet_floor_dof_rootd_Aa001_t_z_up"],
        ["", "_free_13_cabinet_floor_dof_rootd_Aa002_t_z_up"],
        ["", "_free_13_cabinet_floor_dof_rootd_Aa003_t_z_up"],
        ["", "_free_13_cabinet_floor_dof_rootd_Aa004_t_z_up"],
        ["", "_free_13_cabinet_floor_dof_rootd_Aa005_t_z_up"]
    ]
}