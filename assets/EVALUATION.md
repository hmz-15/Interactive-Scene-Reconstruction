# Mapping Evaluations

### **Note: due to code change after submitting papers & extra tunning for downstream CAD replacement, the results produced by the current version of code might be slightly different from those reported in papers, while the overall results should be similar.** 

<br />


We evaluate the performance of our mapping module on the [SceneNN](https://hkust-vgd.github.io/scenenn/) dataset. We picked 20 sequences (scenes) with diverse categories of objects for evaluation; the list of sequence ids are below.  

["005", "011", "016", "030", "061", "062", "069", "078", "086", "096", "202", "223", "225", "231", "249", "276", "286", "294", "322", "521"]

We propose to evaluate on three sub-tasks, either on sequence-level or class-level (on all 20 sequences): 
- 3D panoptic segmentation (PQ, SQ, RQ)
- 3D instance segmentation (AP)
- 3D oriented bounding box estimation (AP)



## 1. Usage

### 1.1 Run Mapping Experiments in Batch

We prepare a simple scipt to produce results for many sequences at a time.

**Step 1**: Same to Step 1 of [running mapping](../mapping/README.md). Open the first terminal, and launch the python3-based image segmentation server.

``` shell
conda activate robot-scene-recon
cd Interactive-Scene-Reconstruction/mapping/rp_server
python launch_detectron_server.py
```

**Step 2**: Similar to Step 2 of [running mapping](../mapping/README.md). Launch the ros nodes in another terminal. There is no need to specify sequence name as it will be frequently modified by external service. You probably want to disable visualization to save resources.

``` shell
roslaunch panoptic_mapping_pipeline scenenn_pano_mapping.launch visualize_mapping:=false visualize_fusion_seg:=false
```

**Step 3**: Open the third terminal, and run the [provided script](../run_exp.sh) to iteratively run mapping for a list of sequences. Please see the script for details; **minor changes are needed** to adapt to your project path and rosbag path. "Done" will appear on the sceen when all experiments are done running. 

``` shell
# Change according to the shell you use
zsh run_exp.sh
```

The results will be saved in subfolders under your output folder.

### 1.2 Run Evaluations

The evaluation code is in the [map_proc](../cad_replacement/map_proc) package for ease of implementation. So please build the package beforehand (see [INSTALL](INSTALL.md)).

Then launch the evaluation node. See the [launch file](../cad_replacement/map_proc/launch/eval.launch) and an extra [yaml file](../cad_replacement/map_proc/cfg/eval.yaml) for details.

``` shell
roslaunch map_proc eval.launch
```

The computed metrics will be saved as a json file in your output folder, while printed on the screen.



## 2. Evaluation Results

We run mapping **5** times each sequence, and report the averaged results below. As shown in the shell script we provided, the rosbag playback speed is decreased by **0.5** by default. 

We are aware that there are two versions of SceneNN annotations:

- One version comes with the [original released dataset](https://hkust-vgd.ust.hk/scenenn/main/), with part of sequences have [nyu_class annotations](https://hkust-vgd.ust.hk/scenenn/contrib/nyu_class/)
- Another version including instance segmentation of 76 scenes ([sceneNN_76](https://drive.google.com/file/d/1DcKKrdzirUjKejjasexGdU0ECMpIXCD3/view)) comes with [a later paper](https://github.com/hkust-vgd/pointwise), and is used more for training & testing of 3D deep learning

The two versions of annotations are slightly different in terms of the instances they annotate, and the exact annotated mesh segment for each instance. We observed that varied results were produced using different versions, especially for object categories that have few instances.

### 2.1 Results using Annotation Version 1

In our papers, we report the results using the first version of annotations (and use nyu_class if available for better consistency). Below tabulates the class-averaged results on all 20 sequences; for sequence-level results please refer to the [raw json file](all_result_raw.json). Due to code changes and parameter tuning, the results is slightly better than those reported for most metrics & categories.

|||all|wall|floor|bed|table|chair|monitor|sofa|bag|cabinet|fridge|
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|Panoptic Seg| PQ|38.2| 23.2|54.6|18.2|36.4|41.5|68.5|43.4|9.8|23.5|62.9|
|| RQ | 47.6|33.1|69.7|24.3|48.1|52.0|77.7|53.5|11.4|31.9|74.4|
|| SQ | 74.2|70.2|78.3|75.7|75.8|79.9|88.1|81.2|34.1|74.7|84.1|
|Instance Seg| MAP| 59.6|-|-|37.6|57.5|66.2|74.6|77.4|22.5|44.4|96.3|
|OBB (un-refined)| MAP |43.3|-|-|32.6|46.6|44.7|47.9|65.5|11.3|34.0|63.8


### 2.2 Results using Annotation Version 2

We also report the results using the scenenn_76 annotations in [raw json file](all_result_76.json) (we still use version 1 annotation for 2 sequences not included in the 76 sequences). The results are worse than those using the first version of annotations, but we observed that **the discussions and comparisons in the paper are still valid**.


