#!/bin/bash

CWD="$(pwd)"

gnome-terminal -x bash -c " 
			   source  ${CWD}/devel/setup.bash;
			   rosservice call gazebo/delete_model '{model_name: startboxsign_}';"



exit 0

