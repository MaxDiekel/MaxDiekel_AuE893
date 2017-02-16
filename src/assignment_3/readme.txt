Max Diekel
Assignment 3 Readme

Assignment 3
Due 2/16/17

There are two things that I have included with this submission. First I included a python script [named turtlebot_goal_around_objects.py] in ./src/srcipts/. This script subscribes to the odometry topic and the scanner topic and uses this information to guide the turtlebot to the goal. This goal is at location (4,4) by default but it can be changed by editing the value in lines 34 and 35 which change the goal x and y coordinate respectively. I have also added a world file which has a jersey barrier setup. This world file was created in such a way that the default goal location demonstrates the code quite well. 

Launch this world file by typing:
$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:= <world file's path>jersey_barrier_obstacle.world

Then run the python script by changing to the directory where the script is then typing:
$ python turtlebot_goal_around_objects.py

If the script is not an executable after downloading it from github, make it an executable by going to the directory where the script is and typing:
$ chmod +x turtlebot_goal_around_objects.py

Then try running the script again. 

What the script does:
The script subscribes to the scan topic and the odom topic and uses this information. When there is no object detected in front of the turtlebot, the script publishes twist messages telling the turtlebot to go towards the goal. The speed and orientation of the robot are controlled with two independent PID controllers. The maximum speed and rotation of the robots were also capped in this move_goal mode. When the object detects an object in front of it, it exits the move_goal function and rotates until there is nothing in its field of view. Then it starts to move toward the goal again. While this algorithm is very simple, it does work quite well and adequately reaches the goal in most cases. However, it does not handle moving directly in the -x (world coordinates). This is due to the atan2() issue, and I did not incorporate a wrapping solution. 

Thanks,
Max
