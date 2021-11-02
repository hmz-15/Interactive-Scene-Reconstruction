import sys
import os


from global_settings import MESHLAB_COLOR_TO_TEXTURE_TEMPLATE
from global_settings import TMP_MESHLAB_SCRIPT
from global_settings import VERTEX_TO_FACE_TEXTURE_DIM
from global_settings import MESHLAB_TEXTURE_FILE_TEMPLATE
from global_settings import MESHLAB_EXE_TEMPLATE_SCRIPT
from global_settings import MESHLAB_EXE_TEMPLATE
from utils import print_warn, print_err, print_info


class MeshlabServer(object):

    def __init__(self):
        pass


    def convert_obj_to_dae(self, file_in, file_out):
        mesh_output_config = "vc vn fc wt"
        cmd = MESHLAB_EXE_TEMPLATE.format(file_in, file_out, mesh_output_config)

        print("Processing `{}`, waiting...".format(cmd))
        os.system(cmd)

        if os.path.exists(file_out):
            print("[INFO] Convert .obj to .dae at: `{}`".format(file_out))
        else:
            print("[ERROR] Fail to convert .obj to .dae: `{}`".format(file_in))
            raise


    def convert_ply_to_obj(self, file_in, file_out):
        input_dir = "/".join(file_in.split("/")[:-1])
        output_dir = "/".join(file_out.split("/")[:-1])
        filename = file_out.split("/")[-1].split('.')[0]

        text_dim = VERTEX_TO_FACE_TEXTURE_DIM
        texture_filename = MESHLAB_TEXTURE_FILE_TEMPLATE.format(filename)

        while True:
            try:
                # generating the meshlab script
                script = self.load_script_template_(MESHLAB_COLOR_TO_TEXTURE_TEMPLATE)
                script = script.format(text_dim, texture_filename)
                script_dir = self.dump_script_template_(script, TMP_MESHLAB_SCRIPT)

                mesh_output_config = "vc vn wt"
                cmd = MESHLAB_EXE_TEMPLATE_SCRIPT.format(file_in, file_out, mesh_output_config, script_dir)

                print("Processing `{}`, waiting...".format(cmd))
                os.system(cmd)
                
                # mv meshlab generated texture file file to dump out directory
                assert(os.system("mv {}/{} {}/".format(input_dir, texture_filename, output_dir)) == 0)
                # remove temporary meshlab script
                assert(os.system("rm {}".format(script_dir)) == 0)

                # no error happened jump out of the loop
                break
            except:
                print_warn("[WARN] Fail to convert .ply to .obj: `{}`".format(file_in))

                if text_dim < 8000:
                    text_dim = int(text_dim * 1.5)
                    print_warn("Increase text dimension to `{}`".format(text_dim))
                else:
                    break

        if os.path.exists(file_out):
            print_info("[INFO] Convert .ply to .obj at: `{}`".format(file_out))
        else:
            print_err("[ERROR] Fail to convert .ply to .obj: `{}`".format(file_in))
            raise


    def load_script_template_(self, script_dir):
        script = ""

        with open(script_dir, "r") as fin:
            for line in fin:
                script += line
        
        return script

    
    def dump_script_template_(self, script, out_dir):
        with open(out_dir, "w") as fout:
            fout.write(script)

        return out_dir
        