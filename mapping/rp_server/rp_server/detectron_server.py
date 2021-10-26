import json
import zlib
import time

import numpy as np

from rp_server import TcpServer
from rp_server import DetectronWrapper
from rp_server import bin2img, bin2int, int2bin


def magic(idx):
    foo = lambda x: idx + 1 if x else 0
    return lambda x: map(foo, x)


def pack_instseg_(resp):
    """
    Pack detectron2 (Inst_seg) result data into binary package
    """
    boxes = resp.pred_boxes.tensor.cpu().detach().numpy().tolist()
    scores = resp.scores.cpu().detach().numpy().tolist()
    classes = resp.pred_classes.cpu().detach().numpy().tolist()
    masks = resp.pred_masks.cpu().detach().numpy()

    # sent a 0 size package
    if boxes is None:
        return int2bin(0)

    # fused all masks into a single mask
    # mask_id = idx + 1
    start_t = time.time()
    n_inst, h, w = masks.shape
    fused_masks = np.zeros(shape=(h, w), dtype="uint8")

    # start_t = time.time()
    for idx in range(n_inst):
        for i in range(h):
            for j in range(w):
                if masks[idx, i, j]:
                    fused_masks[i, j] = idx + 1

    # for idx in range(n_inst):
    #     map(magic(idx), masks[idx])
    #     # a = [[*m] for m in [*map(magic(idx), masks[idx])]]
    #     # fused_masks += np.array([list(m) for m in list(map(magic(idx), masks[idx]))], dtype="uint8")
    # print("Time elapsed: {}".format(time.time() - start_t))

    compressed_mask_bin = zlib.compress(fused_masks.tobytes())
    mask_size_bin = int2bin( len(compressed_mask_bin) )
    mask_pkg = mask_size_bin + int2bin(w) + int2bin(h) + compressed_mask_bin

    info_json = {
        "boxes": boxes,
        "scores": scores,
        "classes": classes
    }
    info_bin = json.dumps(info_json).encode()

    pkg_size = int2bin(len(mask_pkg) + len(info_bin))

    return pkg_size + mask_pkg + info_bin

    
def pack_panoseg_(resp):
    """
    Pack detectron2 (Pano_seg) result data into binary

    | pkg_size (4B int) | map_size (4B int) | width (4B int) | ...
    | height (4B int) | binary_map (map_size B) | json_info_binary (rest) |
    """
    # pack segmentation map
    pano_resp = resp["panoptic_seg"]
    inst_resp = resp["instances"]
    seg_map = pano_resp[0].cpu().detach().numpy().astype("uint8")
    h, w = seg_map.shape
    compressed_map_bin = zlib.compress(seg_map.tobytes())
    map_size_bin = int2bin( len(compressed_map_bin) )
    map_data_pkg = map_size_bin + int2bin(w) + int2bin(h) + compressed_map_bin
    
    # pack semantic information
    info_json = {
        "info": pano_resp[1],
        "boxes": inst_resp.pred_boxes.tensor.cpu().detach().numpy().tolist()
    }
    info_bin = json.dumps(info_json).encode()

    # total package size
    print(len(map_data_pkg))
    pkg_size = int2bin(len(map_data_pkg) + len(info_bin))

    return pkg_size + map_data_pkg + info_bin


DT_ENCODER = {
    "Pano_seg": pack_panoseg_,
    "Inst_seg": pack_instseg_
}
    

class DetectronServer(TcpServer):

    def __init__(self, host, port, model_type="Pano_seg"):
        super(DetectronServer, self).__init__(host=host, port=port)
        
        self.dt_ = DetectronWrapper(task=model_type)
        self.model_ = model_type


    def handle_connection_(self, conn, addr):
        conn_id = "{}:{}".format(addr[0], addr[1])
        print('New connection from {}'.format(conn_id))

        while not self.quit_event_.is_set():
            pack_size = conn.recv(4)
            
            # end of Connection
            if not pack_size:
                break

            pack_size = bin2int(pack_size)
            # fetch data package
            data = self.recv_all_(conn, pack_size)

            img = bin2img(data)
            ret = self.dt_.predict(img)
            
            # send back response
            conn.sendall( self.pack_(ret, self.model_) )

        conn.close()
        print("Connection {}: closed".format(conn_id))


    def pack_(self, resp, model_type):
        if model_type in DT_ENCODER:
            return DT_ENCODER[model_type](resp)
        else:
            raise Exception("Does not support type: {}".format(model_type))

    
if __name__ == "__main__":
    server = DetectronServer(host="192.168.1.94", port=8801, model_type="Pano_seg")
    server.launch()



# pkg = {
#     "boxes": boxes,
#     "scores": scores,
#     "classes": classes,
#     # "masks": masks
# }