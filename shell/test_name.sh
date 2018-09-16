

rosbag_dir="/home/hzhang8/work/data/tum_vio"
# declare -a rosbag_name_list=("room1_512_16 room2_512_16")
rosbag_name_list="corridor1 corridor2 corridor3 corridor4 corridor5 magistrale1 magistrale2 magistrale3 magistrale4 magistrale5 magistrale6 outdoors1 \
		outdoors2 outdoors3 outdoors4 outdoors5 outdoors6 outdoors7 outdoors8 room1 room2 room3 room4 room5 room6 slides1 slides2 slides3"

# i=1
for name in $rosbag_name_list
do
    fname="$rosbag_dir/dataset-${name}_512_16.bag"
    # j=$((i+5))
    # echo "i= $i j=$j"
    if [ -f "$fname" ]
    then
	echo "good found $fname"
    else
	echo "bad!!!!!! not found $fname "
    fi
done
