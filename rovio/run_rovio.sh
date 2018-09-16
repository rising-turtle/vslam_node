
#!/bin/bash

# David Z, Sep 11, 2018 (hzhang8@vcu.edu)
# 
# run rovio with given bagfile and times 
# ./run_rovio.sh  
#

# rosbag_name_list="corridor1 corridor2 corridor3 corridor4 corridor5 magistrale1 magistrale2 magistrale3 magistrale4 magistrale5 magistrale6 outdoors1	outdoors2 outdoors3 outdoors4 outdoors5 outdoors6 outdoors7 outdoors8 room1 room2 room3 room4 room5 room6 slides1 slides2 slides3"

# rosbag_name_list="room3 room4 room5 room6 corridor1 corridor2 corridor3 corridor4 corridor5 magistrale1 magistrale2 magistrale3 magistrale4 magistrale5 magistrale6"

# rosbag_name_list="magistrale2 magistrale3 magistrale4 magistrale5 magistrale6"
# rosbag_name_list="slides1 slides2 slides3"
rosbag_name_list="outdoors1 outdoors2 outdoors3 outdoors4 outdoors5 outdoors6 outdoors7 outdoors8"

times=1

cur_dir=`pwd`

rosbag_dir="/media/davidz/Seagate Backup Plus Drive/dataset/tum_vio"
roslaunch_file="rovio_rosbag_node_tum_param.launch"
result_dir="$cur_dir/../result"
rosbag_file=""
exec_dir="$cur_dir"

do_it(){
    i=$1
    cd $exec_dir
    echo "roslaunch $roslaunch_file bag_file:=$rosbag_file"
    roslaunch $roslaunch_file bag_file:="$rosbag_file" # >/dev/null 2>&1

    # wait for roslaunch start 
    sleep 1

    ### process the result 
    cd $result_dir
    echo "handle $result_dir"
    if [ ! -d "tum_vio_result/$rosbag_name/rovio" ]; then
	mkdir -p "tum_vio_result/$rosbag_name/rovio"
    fi
    
    ### convert from bag to log
    ../../../devel/lib/vslam_rovio/traj_bag_to_log "rovio_result.bag" "rovio_result.log"
    rm "rovio_result.info"
    mv "rovio_result.log" "tum_vio_result/$rosbag_name/rovio/result_$i.log"

    echo -ne '\n'
}


# for name in "${rosbag_name_list[@]}"
for n in $rosbag_name_list
do
    rosbag_name="${n}_512_16"
    rosbag_file="$rosbag_dir/dataset-$rosbag_name.bag"

    i=1
    while [ $i -le $times ]; do
	echo "rovio $i"
	do_it $i
	i=$((i+1))
	sleep 2
    done
    cd $cur_dir

done

echo "finish the job, return to $cur_dir"
cd $cur_dir
exit 0
