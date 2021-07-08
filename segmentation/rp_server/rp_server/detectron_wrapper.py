import detectron2
from detectron2 import model_zoo
from detectron2.config import get_cfg
from detectron2.engine import DefaultPredictor


CONFIG_FILE = {
    # "Pano_seg": "COCO-PanopticSegmentation/panoptic_fpn_R_101_3x.yaml",
    "Pano_seg": "Misc/panoptic_fpn_R_101_dconv_cascade_gn_3x.yaml",
    "Inst_seg": "COCO-InstanceSegmentation/mask_rcnn_R_101_FPN_3x.yaml",
    "Obj_detect": "COCO-Detection/faster_rcnn_R_101_FPN_3x.yaml"
}

DETECTION_RES_KEY = {
    "Pano_seg": "panoptic_seg",
    "Inst_seg": "instances",
    "Obj_detect": "proposals"
}


class DetectronWrapper(object):

    def __init__(self, task="Pano_seg", score_thresh=0.7):
        self.task_ = task
        cfg = get_cfg()
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = score_thresh

        try:
            assert(task in CONFIG_FILE)
            cfg_file = CONFIG_FILE[task]
            cfg.merge_from_file(model_zoo.get_config_file(cfg_file))
            cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(cfg_file)
        except:
            raise Exception("Do not support type `{}`".format(task))
            
        self.predictor_ = DefaultPredictor(cfg)
    

    def predict(self, img):
        ret = self.predictor_(img)

        # return ret[ DETECTION_RES_KEY[self.task_] ]
        if self.task_ == "Pano_seg":
            return ret
        else:
            return ret[ DETECTION_RES_KEY[self.task_] ] 


    