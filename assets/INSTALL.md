# Installation on Ubuntu

## Prerequisites

- Ubuntu 16.04/18.04/20.04 with compatible ROS version
- Python >= 3.7
- gcc & g++ >= 5.4
- OpenCV 3 or 4
- Nvidia GPU (with compatible cuda toolkit and cuDNN) if want to run online segmentation
- Anaconda for configuring python dependencies

## Clone the repository & install catkin dependencies

First create and navigate to your catkin workspace

``` shell
cd <your-working-directory>
mkdir <your-ros-ws>/src && cd <your-ros-ws>
```

Then, initialize the workspace and configure it. (Remember to replace <your-ros-version> by your ros version)

``` shell
catkin init
catkin config --extend /opt/ros/<your-ros-version> --merge-devel 
catkin config --cmake-args -DCMAKE_CXX_STANDARD=14 -DCMAKE_BUILD_TYPE=Release
```
  
Download this repository to your ROS workspace `src/` folder with submodules via:

``` shell
cd src
git clone --recursive https://github.com/hmz-15/Interactive-Scene-Reconstruction.git
```

Then add dependencies specified by .rosinstall using wstool. For your conivenience, we provide a shell to automatically setup the wstool configuration.

``` shell
cd <your-ros-ws>
sh src/Interactive-Scene-Reconstruction/wstool_setup_https.sh
```

Noted that switch to `wstool_setup_ssh.sh` if you are using ssh instead of https.

## Install python dependencies

We assume using [conda virtual environment](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html#activating-an-environment) to configure python dependencies. This requires [Anaconda](https://www.anaconda.com/products/individual) to be installed and initialized as prerequisite. We create a conda env with `python3.7` and install the dependencies:

``` shell
conda create --name robot-scene-recon python=3.7 -y
conda activate robot-scene-recon
pip install pip --upgrade

cd <your-ros-ws>/src/Interactive-Scene-Reconstruction
# detectron2 dependencies
pip install torch torchvision
python -m pip install detectron2 -f \
  https://dl.fbaipublicfiles.com/detectron2/wheels/cu102/torch1.10/index.html
# other dependencies
pip install -r requirements.txt
```

Note that you may need to adjust the version of [torch, torchvision](https://pytorch.org/) and [detectron2](https://detectron2.readthedocs.io/en/latest/tutorials/install.html) based on your cuda/cuDNN version.

You can deactivate the conda env using:
``` shell
conda deactivate
```


## Build packages

We first build the python package containing panoptic segmentation server.
``` shell
conda activate robot-scene-recon
cd Interactive-Scene-Reconstruction/mapping/rp_server
make dev
```

Then we build the ros packages with `catkin build`.
``` shell
cd <your-ros-ws>
# Build packages for panoptic mapping
catkin build panoptic_mapping_pipeline scene_builder gazebo_simulation -j2
source devel/setup.bash
```
Please replace `bash` by `zsh` if `zsh` is your default shell.
  