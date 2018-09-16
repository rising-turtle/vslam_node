#!/bin/bash

# David Z, Sep 9, 2018 (hzhang8@vcu.edu)
# 
# run vins-mono with given bagfile and times 
# ./run_vins-mono.sh [bag_dir] [bagfile] [times] 
#

cur_dir=`pwd`

# rosbag_file="/home/davidz/work/data/drone/dataset_3/rgbd_imu.bag"
# rosbag_file="/home/hzhang8/work/data/tum_vio/dataset-room4_512_16.bag"
rosbag_dir="/home/hzhang8/work/data/tum_vio"
rosbag_name="room4_512_16"
roslaunch_file="$cur_dir/../VINS-mono/launch/tum_vio.launch"
result_dir="$cur_dir/../result"

times_=2

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

rosbag_file="$rosbag_dir/dataset-$rosbag_name.bag" 

do_it(){
    i=$1
    echo "roslaunch $roslaunch_file"
    roslaunch $roslaunch_file & >/dev/null 2>&1

    # Save progress() PID
    # You need to use the PID to kill the function
    ROS_PID=$!

    # wait for roslaunch start 
    sleep 5 

    echo "rosbag play $rosbag_file"
    # rosbag play -u 30 $rosbag_file --r=0.85 >/dev/null 2>&1
    rosbag play $rosbag_file --r=0.85 >/dev/null 2>&1
    echo "finish rosbag play!"

    # Kill progresse
    echo "kill pid $ROS_PID"
    kill $ROS_PID >/dev/null 2>&1
    
    # wait for stop 
    sleep 1

    ### process the result 
    cd $result_dir
    echo "handle $result_dir"
    if [ ! -d "tum_vio_result/$rosbag_name/vins-mono" ]; then
	mkdir -p "tum_vio_result/$rosbag_name/vins-mono"
    fi
    cp "vins_result_no_loop.csv" "tum_vio_result/$rosbag_name/vins-mono/result_$i.log"

    echo -ne '\n'
}

i=1
while [ $i -le $times_ ]; do
    echo "vins-mono $i"
    do_it $i
    i=$((i+1))
    sleep 2
done

echo "finish the job, return to $cur_dir"
cd $cur_dir

exit 0



