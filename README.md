# Guiding the Turtlebot Using Signs Along a Maze From Start To End

Project Name: Mazebot

Team Members:
- Pushkaraj Palnitkar
- Obe Joseph

---

## Project Description
### Youtube Preview : https://youtu.be/elzXXDgsHss
A maze environment was created for Gazebo as shown in Fig. 1.  The turtlebot was placed at the start of the maze. The walls of the maze are labelled with simple instructions like 'Left', 'Right', and ' U-turn'.  The robot will use optical character recognition to *read* the signs, and perform the actions corresponding to the insturctions on the signs. The robot will navigate to the end of the maze.

![maze_view](/home/student/catkin_ws/src/mazebot/Images/maze_top.png  "maze_view")
Fig. 1 Maze Environment with Signs Placed in Appropriate Positions


### Contributions

This requires color and text recognition to control twist commands and therefore, the robot movement.
It is also necessary to get the bot to achieve a specific change in angular position for the turns.
It will be necessary to adjust the perspective of images to be able to read the text from an angle.

---

## ROS nodes in the project

1) Master Node - It launches the Gazebo simulator with the environment created by the .world file.

2) img_to_str Node - This node is in the 'img_str.py' file. It takes input(images) from the camera feed taken from  /camera/rgb/image_raw topic. Since pytesseract has limitations with the different lighting conditions and viewing angles we converted image into the binary(black_white masked) image and then passed it to the pytesseract function to extract text from the image. Python-tesseract (pytesseract)  is an optical character recognition (OCR) tool for python.

3) str_to_twist Node - This node is in the 'str_twist.py' file. The purpose of this node is making robot move based on the information given by the 'scan', 'img_to_str', and '/odom' ROS topics. This node is subscribed to all these three ROS topics. It then publishes Twist commands that makes our robot move and complete the task of coming at the end of the maze from a start point.

![Flow](/home/student/catkin_ws/src/mazebot/Images/Untitled.png  "Flow")

---

## Installation Instructions

- Open a new terminal for this section.

### Install tesseract / pytesseract for OCR:
```
sudo apt update sudo apt install tesseract-ocr
sudo pip install pytesseract
```

### Create the package:
```
cd ~/catkin_ws/src
catkin_create_pkg mazebot rospy geometry_msgs sensor_msgs std_msgs nav_msgs
```

- Download the "mazebot" zip and extract the mazebot folder
- Open the mazebot folder and select all the contents of the folder.
- Copy and paste the selected contents in "/home/student/catkin_ws/src/mazebot" (the mazebot package you created)

### Make our Python scripts executable:
```
cd ~/catkin_ws/src/mazebot/code/mazebot/scripts
chmod +x *.py
```

### Compile/make our package:
```
cd ~/catkin_ws
catkin_make
```

### Fixing Gazebo bug:
- Use any browser to go to [https://github.com/safijari/gazebo_depth_bugfix](https://github.com/safijari/gazebo_depth_bugfix) 
- Download "gazebo2_14_04_indigo.tar.gz" and extract the folder.
```
cd /home/student/Downloads/gazebo2_14_04_indigo
sudo mv * /usr/lib/x86_64-linux-gnu/
```

---

## Running the Code
- You are going to need 3 terminals for this. You can continue using the terminal from before. And open 2 new tabs.

### Launch Gazebo and place the turtlebot in a maze:
```
cd ~/catkin_ws/src/mazebot/code/mazebot/scripts
roslaunch mazebot course.launch
```

###OCR & See robot view:
- Open the second tab in terminal
```
cd ~/catkin_ws/src/mazebot/code/mazebot/scripts
rosrun mazebot img_str.py
```

###Let robot navigate the maze:
- Open the third tab in terminal
```
cd ~/catkin_ws/src/mazebot/code/mazebot/scripts
rosrun mazebot str_twist.py
```

---


## Extra capability: Generating signs (txt_img.py)

This python code is a furher advancement to make project as efficient as possible. Here, the python program takes user input in the form of the string through the command terminal and then returns the .png file into the folder. We used Python Imaging Library(PIL) to create a blank white 1024x1024 image and then draw text on that blank image.

To create new instructions(image) please type the following in a new  terminal -
```
cd ~/catkin_ws/src/mazebot/code/mazebot/scripts
python txt_img.py "<insert instruction text here>"
```
Then check the directory for the newly created png file with the inserted instruction text.

## Measures of Success

<TABLE>
<TR>
	<TH>Measure of Success (from your PROPOSAL)</TH>
	<TH>Status (completion percentage)</TH>
</TR>
<TR>
	<TD>Tesseract library can be used successfully for OCR</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>Maze constructed</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>Bot can identify instructions image by color</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>Bot can convert instructions on image to usable strings</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>The string is parsed in a logical manner</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>The bot can follow the string and move accordingly</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>The bot makes it from start to end</TD>
	<TD>100%</TD>
</TR>
</TABLE>


---

## What did you learn from this project?

- We now have a much better understanding of the launch file and its relationship with its components (like kobuki.launch.xml and world file)
- We can manipute the robot's angular and linear position by editting kobuki.launch.xml
- We have a better understanding of how to structure the world file and change its components' attributes
- We are aware of gazebo's different materials
- We can now creaate our own material file and materials
- We got more familiar with the GIMP Image Editor when creating the custom signs for the maze world.
- We used the nav_msgs to get odometry information
- We used Tesseract for OCR
- We understand the class structure better in PYTHOn
- We understand SELF AND \__INIT__ METHOD IN PYTHON CLASS
- We understand more rospy functions (like wait_for_message)

- The first challenge was the virtual machine depth ordering bug.  It caused the camera sensor to output an image where some objects rendered transparent. To fix this we used a repository of pre-compiled binaries as we saw in the installation instructions.
- The second problem was the drifting of the robot when we gave it a linear velocity along the x axis. To combat the drift we give the robot a tiny angular velocity as it moves forward (0.275 m/s with 0.004 rad/s).  This velocity is removed when the robot is turning at a corner, so that it does not confict with the velocity needed for turning.
	- A better way of ensuring that the robot moves in a straight line would be to use sensor information. We will use range information to ensure that the robot is always at an equal distance from the maze walls to the right and left, when it is moving. This will give us much better movement that applying an abitruary angular velocity with forward movement.
- The third problem was the means of rotating the robot. We first tried giving the robot certain anglur velocities for turning. This would cause the robot to sometimes run into the walls of the maze, as it would not complete its turn at an appopriate angular position. To solve this we used euler_from_quaternion, quaternion_from_euler to constantly check the angular position of the robot. This allowed us to outline when the angular velocity should start and stop being applied.

---

## Future Work

- We have been able to programmatically make .png files with desired text drawn on it using python library call *PIL*. We should be able to programmatically place these .png files with gazebo's wall materials using python library called *lxml*.

- The robot has been placed at the start of the maze for  each run.  The starting linear and angular position of the robot has been specified for it to be able to complete the maze (in kobuki.launch.xml).  It would be useful if the robot could navigate the maze starting at different yaws or linear positions.

- Also, the signs are pre-made for this maze. It would be useful if the signs could be sent to the robot from user input, to navigate a maze with obstacles. Maybe by using a webcam (https://www.pyimagesearch.com/2018/08/20/opencv-text-detection-east-text-detector/). Or using speech (This package may help: http://wiki.ros.org/gspeech)

- Currently the robot continuosly "reads" from the sensor, and when it is 0.7 meters from a wall, it stops to take action. We can use color detection to let the robot seek signs of a certain color before reading, which will be more useful.

---

## References/Resources

*What resources did you use to help finish this project?*
- Installing Tesseract --> https://www.linux.com/blog/using-tesseract-ubuntu
- Installing Tesseract --> https://www.pyimagesearch.com/2017/07/10/using-tesseract-ocr-python/
- Rospy wait for message --> https://docs.ros.org/diamondback/api/rospy/html/rospy.client-module.html
- Understanding class, init, self --> https://micropyramid.com/blog/understand-self-and-__init__-method-in-python-class/
- For fixing the Gazebo bug --> https://jarisafi.wordpress.com/2016/09/16/fixing-the-gazebo-2-2-virtual-machine-depth-ordering-bug/
- Understanding Gazeboo material customization --> http://gazebosim.org/tutorials?tut=color_model
- Gazebo material list -> http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials
- To make robot rotate in the desired angle of degrees ->
http://www.theconstructsim.com/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry/
- To programmatically create .png file ->
https://code-maven.com/create-images-with-python-pil-pillow
