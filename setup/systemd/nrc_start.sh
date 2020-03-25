#!/bin/bash

# Source ROS
source /opt/ros/melodic/setup.sh

# (Optional) Build nrc_ws
# This should probably be left commented out so as to reduce startup latency.
# cd /home/nrc/nrc_software/nrc_ws/
# catkin_make

# Source devel
# TODO: Make it not require nrc_software in /home/nrc. Also fix in nrc.service
source /home/nrc/nrc_software/nrc_ws/devel/setup.bash

# Launch!
# TODO: Replace with roslaunch once a master launch file is made
roscore