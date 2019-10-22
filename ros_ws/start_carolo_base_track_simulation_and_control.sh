#!/bin/bash

CWD="$(pwd)"

gnome-terminal -x bash -c "source ~/anaconda3/etc/profile.d/conda.sh; 
                           conda activate ros_python; 
			   source  ${CWD}/devel/setup.bash;	
                           export GAZEBO_MODEL_PATH=${CWD}/src/ackermann_vehicle_gazebo/models/carolo_base_track/;
                           roslaunch ackermann_vehicle_gazebo ackermann_vehicle.launch" &

gnome-terminal -x bash -c "rosrun joy joy_node" &
gnome-terminal -x bash -c "source  ${CWD}/devel/setup.bash;
		           rosrun ackermann_drive_teleop joyop.py" &

exit 0

