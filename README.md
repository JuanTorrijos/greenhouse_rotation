# greenhouse_rotation
This is a ROS package that contains scripts of research semester
Note: To work with this package you have to work with Ubuntu 20.04 and ROS Noetic

To install and run the simulation...
1. Follow the instruction here (link) to download and install Jackal simulation package
2. Clone this package in the workspace you download Jackal package
3. Build the workspace
4. Go to this package direction
5. Source ./ setup.bash
6. Roslaunch greenhouse_rotation empty_world.launch
7. Open a new terminal
8. Source ./ setup.bash
9. Rosrun greenhouse_rotation test.py

To run algorithm in a real jackal
1. Connect to jackal via ssh (with its correct credentials)
2. ROS_MASTER_URI="(Jackal IP)"
