#!/bin/bash

# David Z, Sep 14, 2018 (hzhang8@vcu.edu)
# 
# run viorb 
# ./run_okvis.sh [times] 
#

cur_dir=`pwd`
exec_dir="$cur_dir"
result_dir="$cur_dir/../../result"
rosbag_name="room2_512_16"
times_=10

do_it(){
    i=$1
    echo "roslaunch tum_vio_dataset.launch"
    cd $exec_dir
    roslaunch tum_vio_dataset.launch

    sleep 1

    ### process the result 
    cd $result_dir
    echo "handle $result_dir"
    if [ ! -d "tum_vio_result/$rosbag_name/viorb" ]; then
	mkdir -p "tum_vio_result/$rosbag_name/viorb"
    fi
     
    cp "$cur_dir/../result/KeyFrameNavStateTrajectory.txt" "tum_vio_result/$rosbag_name/viorb/result_$i.log"

    echo -ne '\n'
}

i=1
while [ $i -le $times_ ]; do
    echo "viorb $i"
    do_it $i
    i=$((i+1))
    sleep 2
done

echo "finish the job, return to $cur_dir"
cd $cur_dir

exit 0
