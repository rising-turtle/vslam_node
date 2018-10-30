#!/bin/bash

# David Z, Sep 12, 2018 (hzhang8@vcu.edu)
# 
# run okvis with given bagfile and times 
# ./run_okvis.sh [bag_dir] [bagfile] [times] 
#

cur_dir=`pwd`

# rosbag_file="/home/davidz/work/data/drone/dataset_3/rgbd_imu.bag"
# rosbag_file="/home/hzhang8/work/data/tum_vio/dataset-room4_512_16.bag"
rosbag_dir="/home/hzhang8/work/data/tum_vio"
# rosbag_name="room4_512_16"
rosbag_name="corridor2_512_16"
# roslaunch_file="$cur_dir/../rovio/rovio_rosbag_node_tum.launch"
config_file="$cur_dir/../okvis/config/config_okvis_50_20_mono.yaml"
result_dir="$cur_dir/../result"
exec_dir="$cur_dir/../../../devel/lib/vslam_okvis/"

times_=3

if [ $# -ge 1 ]; then
    # rosbag_file=$1
    rosbag_dir=$1
    echo "reset rosbag_dir: $rosbag_dir"
fi
if [ $# -ge 2 ]; then
    rosbag_name=$2
    echo "reset rosbag_name: $rosbag_name"
fi
if [ $# -ge 3 ]; then
    times_=$3
    echo "reset run times: $times_"
fi

# rosbag_file="$rosbag_dir/dataset-$rosbag_name.bag"
data_file_dir="$rosbag_dir/dataset-$rosbag_name/mav0"

do_it(){
    i=$1
    echo "./vslam_okvis_euroc $config_file $data_file_dir"
    cd $exec_dir
    # roslaunch $roslaunch_file bag_file:=$rosbag_file >/dev/null 2>&1
    ./vslam_okvis_euroc $config_file $data_file_dir

    ### handle result 
    sleep 1
    cp result_okvis_euroc.log $result_dir/okvis_result.log

    ### process the result 
    cd $result_dir
    echo "handle $result_dir"
    if [ ! -d "tum_vio_result/$rosbag_name/okvis" ]; then
	mkdir -p "tum_vio_result/$rosbag_name/okvis"
    fi
     
    # j=$((i+5))
    cp "okvis_result.log" "tum_vio_result/$rosbag_name/okvis/result_$i.log"

    echo -ne '\n'
}

i=1
while [ $i -le $times_ ]; do
    echo "rovio $i"
    do_it $i
    i=$((i+1))
    sleep 2
done

echo "finish the job, return to $cur_dir"
cd $cur_dir

exit 0



