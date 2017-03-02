Max Diekel
Assignment 4 Readme

Assignment 4
Due 3/2/17

	There are three things that I have included with this submission. First I included a python script [named lap_runner_3.py] in ./srcipts/. This script subscribes to the mybot's laser scan topic and uses this information to guide the mybot around a track. This track [simpletrack] is also included in ./props/. This "building" has been saved in a world file [simple_track.world] for easy use which is located in the /worlds/ directory. 

	To Use:
		1: Launch the mybot in the in the provided simple_track.world file. 
		2: Make sure that the track is set up reasonably (the robot is near the center of the track)
		3: Run the provided lap_runner_3.py script
		4: Admire the incredible speeds that I managed to accomplish...

What the script does:
	The script subscribes to the scan topic and uses this information to follow a wall. The script publishes twist messages controlling the mybot in such a way to maintain a set distance from the wall to the right of the mybot. The forward motion and the steering of the robot are controlled with two independent PID controllers. The maximum speed and rotation of the robots were also capped due to instability at high speeds. While this algorithm is very simple, it does work and adequately navigates the track most cases. However, it does not do so very quickly. This can be improved greatly, but I am still proud of what I have done. 

Thanks,
Max
