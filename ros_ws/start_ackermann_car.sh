#!/bin/bash

gnome-terminal -x bash -c 'source ~/anaconda3/etc/profile.d/conda.sh; 
                           conda activate ros_python; 
                           roslaunch ackermann_vehicle_gazebo ackermann_vehicle.launch' &

gnome-terminal -x bash -c 'rosrun joy joy_node' &
gnome-terminal -x bash -c 'rosrun ackermann_drive_teleop joyop.py' &

exit 0

