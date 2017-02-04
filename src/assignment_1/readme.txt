Max Diekel
Assignment 1 Readme

Assignment 1
Due 2/2/17

I was unable to properly use the Buffet server after working with the TA's. I made an SSH key, but I am still locked out and unable to push to the remote repository. Because, of this I used github as my version control and my server. I apologize for this. 

After cloning my repository, build the catkin workspace by running the following command from the MaxDiekel_AuE893 directory:

$ catkin_make

This should generate a build and devel directory in the main directory and build two nodes (the other nodes will not build because the CMakeLists.txt does not identify them to need executables). I did this on purpose because those cpp programs are not done.

Next, source the setup.bash file by typing this from the MaxDiekel_AuE893 directory:

$ source /devel/setup.bash

Next, open the turtlesim by running the following commands:

$ roscore  //if you don't have one running already
$ rosrun turtlesim turtlesim_node 

This should open a turtlesim window to view the turtle in a 2D simulation. Now run the cleaning code by calling the assignment_1 package and the robot_cleaner_node node:

$ rosrun assignment_1 robot_cleaner_node

This node publishes a series of messages to a topic which the turtle is subscribed to. This series of messages tells the turtle to go to the bottom right corner of the window, and then cover the window up and down until it reaches the other side. At this point, the turtle returns to its home, the center of the window. You may now close the turtlesim node. 

Next, open the gazebo world with a mobile base in the world. Do this by typing the following command:

$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/opt/ros/kinetic/share/turtlebot_gazebo/worlds/empty.world

This should open an empty world with a mobile base in the center of it. If this does not work, do the following:

$ roslaunch turtlebot_gazebo turtlebot_world.launch  

This should start gazebo with a world with several obstacles inside of it. Delete all of the objects EXCEPT FOR the mobile_base and ground_plane_0.

With an empty world, view the bagged files by changing to the bagfiles/ directory and running the following commands:

$ cd bagfiles/
$ rosbag play go_turn180_comeback.bag

This will publish a series of recorded messages to the topic which the turtlebot is subscribed to. The turtlebot will go forward, turn around and come back (as best as I could do with the my keystroke inputs to the turtlebot teleop node). At this point, reset the gazebo environment by running this service:

$ rosservice call /gazebo/reset_world

Now, play the other bag file. This bag file is a recording of a .cpp code which controls the turtlebot. This .cpp uses an open-loop control strategy by modeling the distance and rotation as a function of linear speed input, and angular speed inputs. As Gregory Dudek and Michael Jenkin, in their Computational Principles of Mobile Robotics, say in chapter 2, without some sensory input for localization, the error associated with this open loop control gets worse and worse over time. This is showcased in the simulator which imparts realist physics (inertia, friction, gravity, etc.). This is why the robot fails to follow the grid perfectly (and gets worse over time). To play this bag file use the following command:

$ rosbag play grid_gazebo.bag

Alternatively, you can run the node which published these recorded messages by running the following command.

$ rosrun assignment_1 robot_cleaner_node_v2

An interesting thing with the bag files is that they do not acheive the same outcome when they are played versus recorded. I do not know the reason for this, but this needs to be looked into more deeply. 

Thank you,
Max
