
# Path Finder

Project Name: Parkbot  

Team Members:
- [Vishesh Singh Thakur, vthakur@buffalo.edu]
- [Ashwin Nair, anair3@gmail.com]

 YouTube video link - https://youtu.be/m_Vm3xRwZ_k
---


## Project Description

*This project aims to build a robot that, when launched in a parking garage, is able to move around between the lanes and is able to detect the presence of an empty or full parking space, and park itself. The project helps the robot to sense its surrounding using its camera, laser sensor and odometer. This sensors helps the robot to find the empty space and then to move forward and park itself. The bot when first enters the parking lot it moves towards the very first parking space, when it is able to mask the lines of a parking space it stops and turns and checks the parking spaces on its left and on its right. If a vehicle is already parked then laser scanner comes into play and lets the robot know the parking spot is taken. So, let the left parking spot is taken and the robot has checked it. The odometer helps the bot know that the left spot has been checked, hence now the right parking spot is checked. If the right parking spot is empty the bot parks itself or else it moves forward to check the other parking spots.*




<img src="https://github.com/IE-482-582/course-project-vishesh_and_ashwin/blob/master/Images/VirtualBox_ubuntu_1404_vm_F18_13_12_2019_12_36_00.png" width="300" height="300" title="Github Logo">

*The parked robot after checking the parking spot.*

<img src="https://github.com/IE-482-582/course-project-vishesh_and_ashwin/blob/master/Images/VirtualBox_ubuntu_1404_vm_F18_13_12_2019_12_34_15.png" width="300" height="300" title="Github Logo">

*The robot checking if the area infront of it is taken or not.*











### Contributions

*There are many programs that use Computer Vision to help navigate a driver while parking their vehicle. This program will be able to help a robot park itself, autonomously, in any parking garage. The logic behind our project is very simple. The robot enters the parking lot, when it sees a parking spot line it masks it and stops. Once, it has stopped it checks the left and the right side for empty parking spaces. The checking is done using laser scanner which is mounted on the robot, if the laser scan does not detect anything infront of the bot then the robot automatically parks itself, otherwise rotates whole 180 degrees on its axis to check the other parking spot. If both parking spots are taken, the bot turns 90 degrees and moves forward.*

---

## Installation Instructions

*For the person running this project should has ROS Indigo installed on Ubuntu 14.04. The programming language used in this project is Python, MS paint was used to draw the basic .png file for the robot to mask the parking space, and gazebo was used to create the environment around the parking space with some models of cars added to simulate a real environment*

List of Prerequisite Software:
- [gazebo] 
- [rviz]

*The Python program imports data such rospy, cv2, cv_bridge, numpy, math, and time. The prgraom also requires to import the data named Images and LaserScan from sensor_msgs.msg package, and Twist data from geometry_msgs.msg for the code to run and collect the data incoming from the robot*

- The first step is to create a package called parkbot in the directory *catkin_ws/src* using the *catkin_create_pkg* command in the terminal window.


					cd ~/catkin_ws/src
					catkin_create_pkg parkbot rospy geometry_msgs sensor_msgs


- The second step is to make a directory called *scripts* inside the *parkbot* package using the command *mkdir scripts*.

					cd ~/catkin_ws/src/parkbot
					mkdir scripts



						
- The third step is to download and copy all the files under the scripts folder into the *scripts* directory.

				git clone https://github.com/IE-482-582/course-project-vishesh_and_ashwin.git
				cd course-project-vishesh_and_ashwin/parkbot
				cp scripts/* ~/catkin_ws/src/parkbot/scripts/

- The fourth step is since, all the copied files must be converted into executables, the user needs to use the terminal window and use the command *chmod +x \*.py* to make all the python scripts executable. Similarly use the same command for making the launch file executable, and use *\*.launch* command, and for the png file use *\*.png*, and use the command * chmod +x kobukiTEST.launch.xml* for the .xml file, *chmod +x course.material* for the .material file, and *chmod +x parking_lot_test.world* to make all the necessary files executable

					cd ~/catkin_ws/src/parkbot/scripts
					chmod +x *.py
				
- The fifth step is to build the package. For this first  the user needs to go to the directory *catkin_ws/src* and use the command *catkin_make* for the package to finish building.

					cd ~/catkin_ws
					catkin_make
- A problem is occuring while we are try to use our course.material file. The *.png* file uploaded is not being read, so to make it work please add the *parking_lot.png* file to the followbots course.material. We are trying rectify this problem till then please do this procedure to make it work.
---

## Running the Code



						
- The first step is to use the command *cd ~/catkin_ws/src/parkbot/scripts* for the terminal to go to that directory, and then use the command *roslaunch course.launch*. This command launches the gazebo world.

					cd ~/catkin_ws/src/parkbot/scripts
					roslaunch parkbot course.launch

- The second step is to open up another terminal window, while keeping the first one open and use the command *rosrun parkbot autopark.py* for sending the robot the script to run.

					cd ~/catkin_ws/src/parkbot/scripts
					rosrun parkbot autopark.py
					
- Now let the bot work around and find the empty space to park itself.
---

## Brief Description about the files

- The *.png* file is to apply a map on to our parking lot. If the user wants to build a map according to another parking lot. They just need to create a *.png* image file which encapsulates the desired parking lot. Once, the *.png* file is created, the user just needs to add it on the course.material file to let the *.png* amalgamate with the world.
- The *.material* file uses the *.png* file and attaches itself to the world. The *.png* files size does not matter because the size of the world can be altered using the *.world* file.
- The *.world* file is what we used to create a real-like simulated environment for the robot to run. The user may add or delete or modify the models in the environment inside this file
- The *.xml* file is used to launch the robot inside the world. The position and orientation of the robot can be changed in this file
- The *.launch* file is used to combine all the data from the *.png* file, the *.material* file, the *.world* file and the *.xml* file and then launch the simulated map inside the gazebo application
- The *.py* is the Python script that controls the robot by sending out the necessary commands, such as checking the range infront of the robot, masking the lines of the parking spot and helping the bot rotate according to the requirement.
- The user may change or modify these files as per need or requirement, for example the user can change the range of the laser scanner we have set for our parking lot or maybe the masking color maybe changed according to the parking lot environment.
---

## Measures of Success
The Measures of Success summary can be seen in the table below:

<TABLE>
<TR>
	<TH>Measure of Success (from our PROPOSAL)</TH>
	<TH>Status (completion percentage)</TH>
</TR>
<TR>
	<TD>Create a simulated environment for the robot</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>Create a map for the robot and apply the map onto the simulated environment</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD> Mounting a camera sensor on a high point to see the whole parking lot </TD>
	<TD>NA</TD>
</TR>	
<TR>	
	<TD> Sending the data from the sensor to the moving robot to help find the empty parking lot space</TD>
	<TD>NA</TD>
</TR>
<TR>	
	<TD> Calibrating the movement of the robot according to the data sent by the camera sensor </TD>
	<TD>NA</TD>
</TR>	
<TR>
	<TD>Spawning the robot in the specific place with right orientation</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>Getting the robot to move between the lines</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>Getting the robot to stop after seeing a point of interest, and rotate left and right to calculate if the parking spot is taken or empty</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>Getting the robot to park itself after seeing an empty spot</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>Getting the robot to navigate around in the entire simulation</TD>
	<TD>75%</TD>
</TR>
</TABLE>

 
- We are still trying to navigate the robot properly between the lines and are having trouble rotating the robot in a specific angle
- We are also trying to make the robot move out of the parking spot after it has been there for a certain amount of time, and navigate out of the parking lot

---

## What did you learn from this project?

- *This project helped us understand how to use to create a simulated world and map a desi
layout on top of it. It also helped us understand how the codes in the .world file work and how to modify them as per need. It helped us gain insight on how to launch our own turtlebot in the environment*

- *This project helped us in understanding the scope of simple wanderbot and turtlebot projects, and how they can be used to do complex tasks such as this*

- *Also while creating the map and setting up the cars and bots we understood alot about how important links are, also how gravity and static works in the whole environment of a gazebo world.*

- *We faced quite a few challenges during our work. We had trouble doing the masking of the map inside the computer vision. We also faced problems in turning the robot in desired angles. We faced some problems while coding multiple if statements inside one another that the robot goes through while navigating. And lastly we had some problems on how to complete one successful navigation of the entire parking space*

- *We are also having problems with the speed of the robot responding to the messages that are sent to it. Because of which sometimes the bot overshots the required task and goes into a loop.*

---

## Future Work


*For the future work, one thing that can make this project applicable for the real world environment is using a camera mounted on place which can cover the whole parking lot or maybe two cameras placed according to the requirement. The camera images can be used to find empty grids using OpenCV and then that data can be processed to find empty parking slots. This is a better idea compared to ours because as soon as a bot enters the parking lot it will get the information about empty space and rather than checking all the spaces to find an empty one it can directly move towards the empty slot. We have found some base codes for this idea and are looking to working towards it in future.*

---

## References/Resources

- We used the concepts learned from the Chapters Wanderbot and Followbot from the ROS book
- We used gazebosim.org and the YouTube channel of 'The Construct', to learn the missing concepts and to create our own code from scratch

