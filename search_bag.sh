source /opt/ros/kinetic/setup.bash

if [ $# -eq 0 ]
then
	echo Input a bag file
fi

for var in "$@"
do
	echo check for reindex
	if [[ $var == *.active ]]
	then 
		echo Reindexing
		rosbag reindex $var 
	fi

	match='match'
	bag='.bag'

	echo Writing data to file
	zebROS_ws/devel/lib/rosbag_scripts/rosbag_scripts_node $var

	echo Is this a match?
	any_data=$(rosbag info $var | grep /frcrobot/match_data)
	if [ ! -z "$any_data" -a "$any_data" != " " ] 
	then
		echo This has match data
		matchNumber=$(sed -n 10p temp_file.txt)
		bag_name=$match$matchNumber$bag
		if [[ -e $bag_name ]]
		then 
			echo This match already has a bag file... merging
			~/2018RobotCode/zebROS_ws/src/rosbag_scripts/scripts/merge_bagfiles.py $bagname $var $bag_name -v 
		else 
			echo Renaming bag file
			cp $var $bag_name
		fi
	else
		echo This does not have match data.
	fi
	#rm temp_file.txt
done


#first, input bag
#second, run search_bag.cpp. it creates a temp file with all match_data text.
#then, search that file for actual data. 
#if returns true, rename the bag to fit the match data. if not, do not rename the bag
