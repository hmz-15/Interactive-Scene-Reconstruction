import sys
import os

import pymeshlab

from global_settings import VERTEX_TO_FACE_TEXTURE_DIM
from global_settings import MESHLAB_TEXTURE_FILE_TEMPLATE
from utils import print_warn, print_err, print_info


class MeshlabServer(object):

    def __init__(self):
        pass


    def convert_obj_to_dae(self, file_in, file_out):
        # convert to absolute path
        abs_file_in = os.path.abspath(file_in)
        abs_file_out = os.path.abspath(file_out)

        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(abs_file_in)

        print("Processing `{}`, waiting...".format(file_in))
        ms.save_current_mesh(abs_file_out)

        if os.path.exists(file_out):
            print("[INFO] Convert .obj to .dae at: `{}`".format(file_out))
        else:
            print("[ERROR] Fail to convert `{}` to .dae format: ".format(file_in))
            raise
    

    def convert_ply_to_obj(self, file_in, file_out):
        # convert to absolute path
        abs_file_in = os.path.abspath(file_in)
        abs_file_out = os.path.abspath(file_out)
        
        # ditrectory configuration
        input_dir = "/".join(abs_file_in.split("/")[:-1])
        output_dir = "/".join(abs_file_out.split("/")[:-1])
        filename = file_out.split("/")[-1].split('.')[0]

        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(abs_file_in)

        # Gives an indication on how big the texture is
        text_dim = VERTEX_TO_FACE_TEXTURE_DIM
        texture_filename = MESHLAB_TEXTURE_FILE_TEMPLATE.format(filename)
        
        while True:
            try:
                ms.parametrization_trivial_per_triangle(
                    sidedim=0,
                    textdim=text_dim,
                    border=2,
                    method=1
                )

                ms.transfer_vertex_color_to_texture(
                    textname=texture_filename,
                    textw=1024,
                    texth=1024,
                    overwrite=False,
                    pullpush=True
                )

                print("Processing `{}` ply to obj, waiting...".format(file_in))
                ms.save_current_mesh(abs_file_out)

                # no error happened jump out of the loop
                break
            except:
                print_warn("[WARN] Fail to convert .ply to .obj: `{}`".format(file_in))

                if text_dim < 8000:
                    text_dim = int(text_dim * 1.5)
                    print_warn("Increase text dimension to `{}`".format(text_dim))
                else:
                    raise

        if os.path.exists(file_out):
            print_info("[INFO] Convert .ply to .obj at: `{}`".format(file_out))
        else:
            print_err("[ERROR] Fail to convert .ply to .obj: `{}`".format(file_in))
            raise
        

def test_meshlab_server():
    meshlab = MeshlabServer()

    file_in = "input/scenenn_225/225/segments/1.ply"
    file_out = "output/test/1.obj"
    meshlab.convert_ply_to_obj(file_in, file_out)

    file_in = "output/test/1.obj"
    file_out = "output/test/1.dae"
    meshlab.convert_obj_to_dae(file_in, file_out)


if __name__ == "__main__":
    test_meshlab_server()