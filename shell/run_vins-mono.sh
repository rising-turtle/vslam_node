#!/bin/bash

# David Z, Sep 9, 2018 (hzhang8@vcu.edu)
# 
# run vins-mono with given bagfile and times 
# ./run_vins-mono.sh [bagfile] [times] 
#

cur_dir=`pwd`

rosbag_file="/home/davidz/work/data/drone/dataset_3/rgbd_imu.bag"
roslaunch_file="$cur_dir/../VINS-mono/launch/robocane_data_no_view.launch"
result_dir="$cur_dir/../VINS-mono"

times=2

if [ $# -gt 1 ]; then
    rosbag_file=$1
fi
if [ $# -gt 2 ]; then
    times=$2
fi

do_it(){
    i=$1
    echo "roslaunch $roslaunch_file"
    roslaunch $roslaunch_file & >/dev/null 2>&1

    # Save progress() PID
    # You need to use the PID to kill the function
    ROS_PID=$!

    # wait for roslaunch start 
    sleep 3 

    echo "rosbag play $rosbag_file"
    rosbag play $rosbag_file --r=0.55  #>/dev/null 2>&1
    echo "finish rosbag play!"

    # Kill progresse
    echo "kill pid $ROS_PID"
    kill $ROS_PID >/dev/null 2>&1
    
    # wait for stop 
    sleep 1

    ### process the result 
    cd $result_dir
    echo "handle $result_dir"
    if [ ! -d "tum_vio_result/vins-mono" ]; then
	mkdir -p "tum_vio_result/vins-mono"
    fi
    cp "vins_result.csv" "tum_vio_result/vins-mono/result_$i.csv"

    echo -ne '\n'
}

i=1
while [ $i -le $times ]; do
    echo "vins-mono $i"
    do_it $i
    i=$((i+1))
    sleep 2
done

echo "finish the job, return to $cur_dir"
cd $cur_dir

exit 0



