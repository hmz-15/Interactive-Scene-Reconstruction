import os
import sys

# import trimesh
# import open3d as o3d

from xacro_scene import XacroScene
from parse_graph import ParseGraph
from meshlab_server import MeshlabServer
from db_loader import DbLoader
from utils import arg_parser
from utils import print_ok, print_warn, print_info, print_err
from obj_type import ObjType

from global_settings import SCENE_PG_FILENAME
from global_settings import SCENE_SEGMENTS_DIR

from global_settings import MAIN_XACRO_FILENAME
from global_settings import LINK_TO_MESH_FILENAME
from global_settings import PARENT_CHILD_PAIR_FILENAME

from global_settings import SCENE_BUILDER_OUTPUT_DIR

from global_settings import RIGID_MESH_FOLDER
from global_settings import INTERACTIVE_MESH_FOLDER
from global_settings import BACKGROUND_MESH_FOLDER
from global_settings import VRGYM_SCALED_MESH_DIR_ROOT

class XacroSceneBuilder(object):

    def __init__(self):
        self.meshlab_ = MeshlabServer()


    def generate(self, config):
        self.config_ = config

        pg = ParseGraph(self.get_pg_file_(self.config_.input_scene_dir))

        xscene = XacroScene(self.config_.output_dir_name, SCENE_BUILDER_OUTPUT_DIR, pg, 
            self.config_.articulated_mesh_default_tf, self.config_.enable_physics, self.config_.enable_gazebo)

        self.add_to_scene_(xscene, pg, pg.get_root_idx())

        saved_dir = xscene.save(self.config_.scene_builder_root)

        # if scene xacro files are succesfully saved
        if saved_dir != "":
            self.dump_rigid_files_(xscene.get_rigid_files())
            self.dump_interactive_files_(xscene.get_interactive_files())
            self.dump_bgm_files_(xscene.get_bgm_files())

            if self.config_.enable_vrgym:
                self.dump_parent_child_pairs_(xscene.get_parent_child_pairs(), saved_dir)
                self.dump_link_mesh_mapping_(xscene.get_link_mesh_mapping(), saved_dir)
                self.dump_scaled_mesh_(xscene.get_link_mesh_mapping(), saved_dir)

            if self.config_.enable_gazebo:
                self.dump_gazebo_pkg_(saved_dir, xscene.get_bgm_files())
            
            print_warn("[TIPS] Use following command to view your reconstructed scene in RViz")
            print_info("roslaunch scene_builder view_scene.launch scene:={}".format(self.config_.output_dir_name))
            print_ok("[OK] Done! `{}` is saved at `{}`".format(self.config_.output_dir_name, saved_dir))
        else:
            print_err("[ERROR] Fail to generate scene `{}`".format(self.config_.input_scene_dir))

    
    def add_to_scene_(self, xscene, pg, idx):
        # add current node into xacro scene
        target = pg.get_node(idx)
        parent = pg.get_parent(idx)

        xscene.add(target, parent)
        
        for c in pg.get_children_idx(idx):
            self.add_to_scene_(xscene, pg, c)

    
    def dump_parent_child_pairs_(self, pc_pairs, out_dir):
        """
        Dump the link to mesh mapping

        The output text file is formed as
            parent_link_name,child_link_name

        @param pc_pairs (a list of pairs): (parent_link_name, child_link_name)
        @param out_dir (string): the output directory of the mapping file
        """
        file_out = "{}/{}".format(out_dir, PARENT_CHILD_PAIR_FILENAME)

        with open(file_out, "w") as fout:
            for pc in pc_pairs:
                fout.write("{},{}\n".format(*pc))


    def dump_link_mesh_mapping_(self, link_to_mesh, out_dir):
        """
        Dump the link to mesh mapping

        The output text file is formed as
            <link-name>,<mesh-name>,<mesh-dir>

        @param link_to_mesh (dictionary): the key is the link_name, and the
            value is the link-mesh info
                {
                    "type": <object-type>,
                    "mesh_dir": mesh directory,
                    "scale": the scale of the mesh
                }
        @param out_dir (string): the output directory root of current scene
        """
        file_out = "{}/{}".format(out_dir, LINK_TO_MESH_FILENAME)

        with open(file_out, "w") as fout:
            for link_name, info in link_to_mesh.items():
                if info["type"] != ObjType.Rigid:
                    continue

                mesh_dir = info["mesh_dir"]
                scale = info["scale"]
                
                # mesh_filename is None when a concept link comes
                if mesh_dir is None:
                    mesh_dir = ""
                
                mesh_filename = mesh_dir.split('/')[-1]
                fout.write("{},{},{},{}\n".format(link_name, mesh_dir, mesh_filename, scale))


    def dump_scaled_mesh_(self, link_to_mesh, out_dir):
        """
        Dump pre-scaled mesh file VRGym


        @param link_to_mesh (dictionary): the key is the link_name, and the
            value is the link-mesh info
                {
                    "type": <object-type>,
                    "mesh_dir": mesh directory,
                    "scale": the scale of the mesh
                }
        @param out_dir (string): the output directory root of current scene
        """
        scaled_rigid_dir = "{}/{}/{}".format(out_dir, VRGYM_SCALED_MESH_DIR_ROOT, RIGID_MESH_FOLDER)
        os.system("mkdir -p {}".format(scaled_rigid_dir))

        for link_name, info in link_to_mesh.items():
            obj_type = info["type"]

            if obj_type == ObjType.Rigid:
                scale = info["scale"]
                mesh_dir = info["mesh_dir"]
                filename = mesh_dir.split('/')[-1]
                db_mesh_file = "{}/{}".format(self.config_.rigid_mesh_db, filename)
                out_mesh_dir = "{}/{}".format(scaled_rigid_dir, filename)

                # mesh = o3d.io.read_triangle_mesh(db_mesh_file)
                # mesh.scale(scale, mesh.get_center())
                # o3d.io.write_triangle_mesh(out_mesh_dir, mesh)

                # print("[INFO] Save scaled mesh at: {}".format(out_mesh_dir))


    def dump_rigid_files_(self, dst_rigid_files):
        if len(dst_rigid_files) == 0:
            return
        
        src_dir = self.config_.rigid_mesh_db

        for file in dst_rigid_files:
            filename = file.split('/')[-1].split('.')[0]

            # output dir wrt to scene_builder
            output_root_dir = "/".join(file.split('/')[:-1])
            
            abs_output_root_dir = "{}/{}".format(self.config_.scene_builder_root, output_root_dir)

            print_info("[INFO] Copying rigid object `{}`".format(filename))

            cmd = "cp {}/{}* {}".format(src_dir, filename, abs_output_root_dir)
            if os.system(cmd) != 0:
                print_err("[ERROR] Fail to dump {}/{}".format(src_dir, filename))
                exit(1)


    def dump_interactive_files_(self, dst_interactive_files):
        if len(dst_interactive_files) == 0:
            return
        
        src_dir = self.config_.articulated_mesh_db

        for file in dst_interactive_files:
            filename = file.split('/')[-1]
            # output dir wrt to scene_builder
            output_root_dir = "/".join(file.split('/')[:-1])
            
            abs_output_root_dir = "{}/{}".format(self.config_.scene_builder_root, output_root_dir)

            print_info("[INFO] Copying interactive object `{}`".format(filename))

            cmd = "cp -r {}/{} {}".format(src_dir, filename, abs_output_root_dir)
            if os.system(cmd) != 0:
                print_err("[ERROR] Fail to dump {}/{}".format(src_dir, filename))
                exit(1)


    def dump_bgm_files_(self, dst_bgm_files):
        if len(dst_bgm_files) == 0:
            return
        
        src_dir = self.config_.input_scene_dir + "/" + SCENE_SEGMENTS_DIR

        # check if background dump directory exists, if not, create it
        dump_bgm_dir = "/".join(dst_bgm_files[0].split('/')[:-1])
        if not os.path.isdir(dump_bgm_dir):
            os.system("mkdir -p {}".format(dump_bgm_dir))
        
        for file_out in dst_bgm_files:
            mesh_filename = file_out.split('/')[-1].split('.')[0]
            file_in = "{}/{}.ply".format(src_dir, mesh_filename)
            abs_file_out = "{}/{}".format(self.config_.scene_builder_root, file_out)

            self.meshlab_.convert_ply_to_obj(file_in, abs_file_out)
        
        print_info("[INFO] Dumped background mesh files at {}".format(dump_bgm_dir))

    
    def dump_gazebo_pkg_(self, scene_output_dir, bg_mesh_files):
        scene_name = scene_output_dir.split('/')[-1]

        # convert background meshes from .obj to .dae for gazebo simulator
        for bgm in bg_mesh_files:
            # xxx.obj --> xxx.dae
            abs_file_in = os.path.normpath("{}/{}".format(self.config_.scene_builder_root, bgm))
            file_out = bgm[:-3] + "dae"
            abs_file_out = os.path.normpath("{}/{}".format(self.config_.scene_builder_root, file_out))
            
            self.meshlab_.convert_obj_to_dae(abs_file_in, abs_file_out)
        
        print_warn("[WARN] Use following command in your ROS environment to generate to gazebo world")
        print_info("roslaunch scene_builder generate_gazebo_world.launch scene_name:={}".format(scene_name))


    def get_pg_file_(self, scene_dir):
        return "{}/{}".format(scene_dir, SCENE_PG_FILENAME)