mkdir -p src/dependency
wstool init src/dependency
wstool merge -t src/dependency src/Interactive-Scene-Reconstruction/mapping/orb_slam2_ros/orb_slam2_ros_ssh.rosinstall
wstool merge -t src/dependency src/Interactive-Scene-Reconstruction/mapping/perception_ros/perception_ros_ssh.rosinstall
wstool merge -t src/dependency src/Interactive-Scene-Reconstruction/mapping/voxblox-plusplus/voxblox-plusplus_ssh.rosinstall
wstool merge -t src/dependency src/Interactive-Scene-Reconstruction/cad_replacement/map_proc/map_proc_ssh.rosinstall
wstool update -t src/dependency