# ETW-Final
Perform the following commands inorder to create a ROS workspace once you have installed ROS melodic an gazebo 9.

: mkdir ~/simulation_ws/src -p

: cd ~/simulation_ws/

(Then paste both the packages in the src folder of the workspace and paste the .dat file inside the workspace)


: source /usr/share/gazebo/setup.sh

: source /opt/ros/kinetic/setup.bash

: catkin_make

: source ~/simulation_ws/devel/setup.bash

Now, that the workspace is created copy paste the 2 packageses inside the 'src' folder of the workspace.

run the following command to compile all the packages

: catkin_make
