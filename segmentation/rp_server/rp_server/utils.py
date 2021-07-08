import struct
import json
import zlib

import cv2
import numpy as np


def img2bin(img):
    """
    Convert cv2 image matrix to binary string

    Args:
        img (cv2::mat): image matrix

    Returns:
        ret (binary string)
    """
    return np.array(cv2.imencode(".jpg", img)[1]).tostring()

    
def bin2img(img_bin):
    """
    Convert binary string to cv2 image matrix

    Args:
        img_bin (binary string): image binary string

    Returns:
        img (cv2::mat): return opencv image matrix
    """
    return cv2.imdecode(np.frombuffer(img_bin, dtype=np.uint8), cv2.IMREAD_COLOR)


def bin2int(int_bin):
    return struct.unpack('>I', int_bin)[0]


def int2bin(integer):
    return struct.pack(">I", integer)


def pack_img(img):
    img_bin = img2bin(img)
    img_size = int2bin( len(img_bin) )

    return img_size + img_bin


def bin2openpose(kp_bin):
    size = len(kp_bin)
    n_poses = size // (3 * 25 * 4)

    if n_poses == 0:
        return []
    
    return np.frombuffer(kp_bin, dtype="float32").reshape((n_poses, 25, 3))


def bin2detectron_instseg(dt_bin):
    """
    Pack detectron2 (Inst_seg) result data into binary

    | pkg_size (4B int) | map_size (4B int) | width (4B int) | ...
    | height (4B int) | binary_map (map_size B) | json_info_binary (rest) |
    """
    if len(dt_bin) == 0:
        return {}
    
    map_size = bin2int(dt_bin[:4])
    w, h = bin2int(dt_bin[4:8]), bin2int(dt_bin[8:12])
    mask_bin = zlib.decompress(dt_bin[12:12+map_size])
    masks = np.frombuffer(mask_bin, dtype="uint8").reshape((h, w))

    dt_res = json.loads( dt_bin[12 + map_size:].decode() )
    
    # add masks field
    dt_res["masks"] = masks
    
    return dt_res


def bin2detectron_panoseg(dt_bin):
    """
    Pack detectron2 (Pano_seg) result data into binary

    | pkg_size (4B int) | map_size (4B int) | width (4B int) | ...
    | height (4B int) | binary_map (map_size B) | json_info_binary (rest) |
    """
    if len(dt_bin) == 0:
        return {}
    
    map_size = bin2int(dt_bin[:4])
    w, h = bin2int(dt_bin[4:8]), bin2int(dt_bin[8:12])
    seg_map_bin = zlib.decompress(dt_bin[12:12+map_size])
    seg_map = np.frombuffer(seg_map_bin, dtype="uint8").reshape((h, w))

    info_json = json.loads( dt_bin[12 + map_size:].decode() )
    print(info_json["boxes"])

    dt_res = {
        "seg_map": seg_map,
        "info": info_json["info"],
        "boxes": info_json["boxes"]
    }
    
    return dt_res


DT_DECODER = {
    "Pano_seg": bin2detectron_panoseg,
    "Inst_seg": bin2detectron_instseg
}


def bin2detectron(dt_bin, model_type="Pano_seg"):
    if model_type in DT_DECODER:
        return DT_DECODER[model_type](dt_bin)
    else:
        raise Exception("[bin2detectron] Does not support model type: ".format(model_type))

