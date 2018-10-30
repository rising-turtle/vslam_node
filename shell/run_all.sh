#!/bin/bash

# David Z, Sep 12, 2018 (hzhang8@vcu.edu)
# 
# run okvis vins_mono vins_mon_ext rovio with datasets in 
# [rosbag_name_list] by 10 [times] 
#

cur_dir=`pwd`

rosbag_dir="/home/hzhang8/work/data/tum_vio"
# declare -a rosbag_name_list=("room1_512_16 room2_512_16")
# corridor1 corridor2 corridor3 corridor4 corridor5 room1 magistrale1 
# rosbag_name_list="room2 room3 room4 room5 room6 magistrale2 magistrale3 magistrale4 magistrale5 magistrale6 outdoors1 outdoors2 outdoors3 outdoors4 outdoors5 outdoors6 outdoors7 outdoors8 slides1 slides2 slides3"

# rosbag_name_list="room1 room2 room3 room4 room5 room6 corridor3 corridor4 corridor5 magistrale1 magistrale2 magistrale3 magistrale4 magistrale5 magistrale6 slides1 slides2 slides3 outdoors1 outdoors2 outdoors3 outdoors4 outdoors5 outdoors6 outdoors7"

# rosbag_name_list="corridor1 corridor2"
# rosbag_name_list="slides1 slides2 slides3"
# rosbag_name_list="outdoors2 outdoors3 outdoors4 outdoors5 outdoors6 outdoors7 outdoors8"

rosbag_name_list="outdoors7"

times=4

# for name in "${rosbag_name_list[@]}"
for n in $rosbag_name_list
do
    name="${n}_512_16"
    echo "handle dataset $name "
    
    # echo "call run_okvis $rosbag_dir $name $times"
    # ./run_okvis.sh $rosbag_dir $name $times
    # echo "finish run_okvis $rosbag_dir $name $times"
    
    echo "call run_vins-mono $rosbag_dir $name $times"
    ./run_vins-mono.sh $rosbag_dir $name $times
    echo "finish run_vins-mono $rosbag_dir $name $times"

    echo "call run_vins-mono_ext $rosbag_dir $name $times"
    # ./run_vins-mono_ext.sh $rosbag_dir $name $times
    echo "finish run_vins-mono_ext $rosbag_dir $name $times"
    
done

# separate since rovio is too slow
for n in $rosbag_name_list
do
    name="${n}_512_16"
    echo "call run_rovio $rosbag_dir $name $times"
    # ./run_rovio.sh $rosbag_dir $name $times
    echo "finish run_rovio $rosbag_dir $name $times"
done
    
