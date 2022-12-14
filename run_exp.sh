# Path to ros workspace
PROJ_PATH=~/Workspace/Interactive-Scene-Reconstruction
# Path to rosbag folders, each rosbag named "scenenn_$SEQ.bag"
DATASET_PATH=~/Data/Datasets/SLAM

cd $PROJ_PATH
# Change to the shell you are using
source devel/setup.zsh

for SEQ in "005" "011" "016" "030" "061" "062" "069" "078" "086" "096" "202" "223" "225" "231" "249" "276" "286" "294" "322" "521"
do
    rosservice call /gsm_node/reset_map "!!str $SEQ"
    sleep 5

    cd $DATASET_PATH
    rosbag play --clock -r 0.5 scenenn_$SEQ.bag
    sleep 100

    cd $PROJ_PATH
    rosservice call /gsm_node/generate_mesh
    sleep 10
    rosservice call /gsm_node/extract_instances
    sleep 10
done
