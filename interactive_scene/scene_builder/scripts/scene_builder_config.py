import os

from utils import load_json
from utils import print_err

from global_settings import SCENE_BUILDER_OUTPUT_DIR


class SceneBuilderConfig(object):

    def __init__(
        self,
        input_scene_dir,
        scene_builder_root,
        output_dir_name,
        rigid_mesh_db,
        articulated_mesh_db,
        articulated_mesh_default_tf_file,
        enable_vrgym=False,
        enable_physics=False,
        enable_gazebo=False
    ):
        self.input_scene_dir = input_scene_dir
        self.scene_builder_root = scene_builder_root
        self.output_dir_name = output_dir_name
        self.output_dir = os.path.normpath("{}/{}/{}".format(scene_builder_root, SCENE_BUILDER_OUTPUT_DIR, output_dir_name))
        self.rigid_mesh_db = rigid_mesh_db
        self.articulated_mesh_db = articulated_mesh_db
        self.articulated_mesh_default_tf = self.load_tf_(articulated_mesh_db, articulated_mesh_default_tf_file)
        
        self.enable_vrgym = enable_vrgym
        self.enable_physics = enable_physics
        self.enable_gazebo = enable_gazebo

        if self.output_dir_name is None:
            self.output_dir_name = self.input_scene_dir.split('/')[-1]

    
    def __str__(self):
        ret = ""

        ret += '#' * 50 + "\n"
        ret += "# Scene Builder Configuration\n"
        ret += '#' * 50 + "\n"

        ret += "* Rigid mesh database: {}\n".format(self.rigid_mesh_db)
        ret += "* Articulated mesh database: {}\n".format(self.articulated_mesh_db)
        ret += '-' * 60 + '\n'
        
        ret += "* Input scene dir: {}\n".format(self.input_scene_dir)
        ret += "* scene_builder pkg root: {}\n".format(self.scene_builder_root)
        ret += "* Output scene dir name: {}\n".format(self.output_dir_name)
        ret += "* Output scene dir: {}\n".format(self.output_dir)
        ret += '-' * 60 + '\n' 
        
        ret += "* Enable VRGym: {}\n".format(self.enable_vrgym)
        ret += "* Enable physics: {}\n".format(self.enable_physics)
        ret += "* Enable Gazebo: {}".format(self.enable_gazebo)
        
        return ret

    
    def load_tf_(self, mesh_db, tf_file):
        interactive_cads = [
            it for it in os.listdir(mesh_db) 
            if os.path.isdir(mesh_db + '/' + it)
        ]

        tf = load_json(tf_file)

        validate_data_format = lambda x, y: x in y and "scale" in y[x] and "tf" in y[x]
        try:
            assert( all([validate_data_format(it, tf) for it in interactive_cads]) )
        except:
            for it in interactive_cads:
                if not validate_data_format(it, tf):
                    print_err("Interactive CAD `{}` was not shown in the transformation file `{}`".format(it, tf_file))
            raise

        return tf