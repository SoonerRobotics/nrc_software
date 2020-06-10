# nrc_software: demo_sim branch

**This branch is not intended ever to be merged into master.**

The demo_sim branch is a stable build of the NRC robot software to be run in simulation for demonstration purposes.
This code is verified to work in the SCR Simulator v.18 with ROS Melodic in Ubuntu 18.04 using WSL 1.

## To run this code with WSL:
 - Clone the repository and switch to the demo_sim branch by running `git checkout demo_sim`.
 - If using WSL, [install VcXsrv](https://sourceforge.net/projects/vcxsrv/) and run it as specified [here](https://janbernloehr.de/2017/06/10/ros-windows#install-vcxsrv).
 - Download [Version 18 of the SCR Simulator](https://github.com/SoonerRobotics/scr_simulator/releases/tag/v18), unzip it, and run the scr_simulator.exe file.
 - From within the repo, run `roslaunch nrc_nav drive_pp_sim.launch`.
 - After the console shows rosbridge has connected to port 9090, click "Run" in the simulator to connect the client, and wait a few seconds for VcXsrv to connect and for the code to start up. If it does not connect the client, press Escape and click "Restart" in the simulator to attempt a reconnect.

## How the robot's path is created:

The course is specified by the file `config.json` in the trajectory_gen folder. This file specifies the _course_ (the first half of the file) and the _waypoints_ (the second half of the file). Course points represent physical nodes such as cones or buckets that the robot needs to go around but not hit. Waypoints are imaginary points that are used to generate the path. Course points are included purely for display purposes, while waypoints are used to generate the trajectory that the robot will follow. The robot is likely to pass through waypoints, so they should be placed a small distance away from the course points to prevent collisions.

## To change the robot's path:
 - Make sure you are in the trajectory_gen folder by running `cd trajectory_gen` from the main directory.
 - Open the config.json file to edit its contents. This can be done by running `nano config.json`, `code config.json`, or whatever editor you prefer. 
 - Modify, add, or remove _waypoints_ to change the path the robot will follow. To ensure an accurate depiction of your setup, you may wish to alter the _course_ points as well so they will be displayed appropriately.
 - Exit the editor and save your changes. 
 - Now that we have a new path, we need to create a new trajectory. Execute `python main.py` to generate a new trajectory. This will replace the file `output_traj.csv` with the new trajectory.
 - The last step is to make the code use this new trajectory rather than the old one. Run `cp output_traj.csv ../nrc_ws/src/nrc_nav/src` to replace the old trajectory with this new one.
 - The next time the code is run, it will generate a path using this new trajectory, and the VcXsrv display should reflect this.
 
