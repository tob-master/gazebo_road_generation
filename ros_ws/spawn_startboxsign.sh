#!/bin/bash

CWD="$(pwd)"

gnome-terminal -x bash -c " 
			   source  ${CWD}/devel/setup.bash;	
			   rosrun gazebo_ros spawn_model -sdf -file /home/tb/.gazebo/models/startboxsign/model.sdf -model startboxsign_ -x 0.0 -y 0.44 -z 0.1 -R -3.14159 -P -1.570795 -Y -1.570795;"


exit 0

