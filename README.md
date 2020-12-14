# ROB599 homework 3
---
## WeitongLiang
### 933955080
### liangw@oregonstate.edu
---

## Introduction

This project mainly realizes a logistics robot that automatically carries goods, which can neatly palletize the scattered goods into the warehouse.

Introduction video: https://www.youtube.com/watch?v=RK5siRriUDg

Video for other task: https://www.youtube.com/watch?v=MTZtKg0TrGQ  


## Project Mission

1. Drive robot to target position function. [5 pts]

2. Service node to randomly generate goods and set task mode. [5 pts]

3. Detection warehouse, using marker to display, save and read data by txt files. [5 pts]

4. Navigation point calculation to goods point. [5 pts]

5. Pick goods function for one goods include move, catch and drop action sequence. Within Pick Action which is a loop for pick all  goods. [10 pts]

6. Basic Task: Palletize all scattered goods correctly in the area. [ 10 pts ]

7. Sort Task: Green goods are placed in the green area, blue goods are placed in the blue area. [ 10 pts ]

8. Layer Task: Green goods on the lower level, Blue goods on the upper level. [10 pts]

## Step

1. This project is based on the Fetch robot, so you need to start fetch world first.
Pre-build rviz file is inside of the project folder, you need to run rviz and open this file to see the simulation process.

2. Start the set_mode.py script, this script is used to select the project mode.

3. Start up show_marker.py script, this script is responsible for publishing markers to display warehouse and goods.

4. Now execute the call service instruction, “rosservice call /set_mode 0”, the input parameter 0 is to start the basic experiment. In this experiment, the robot will move six goods to the warehouse in turn.

5. Run robot_navigation.py, this script include robot’s moving and catching actions.

6. Run pick_client.py, the robot will automatically do its job.

## Precautions

1. The fetch launch file is inside of launch folder, you can run it using “roslaunch” command

2. The files folder have goods data, and script using “rospkg” to access it, make sure you put project folder in right place.

3. The robot movement uses move base action, which is a function of ROS, so it may be a little slow when it running.

4. You can use service call and put in 1 or 2 to start “Sort Task” and “Layer Task”, then you need to restart robot_navigation.py script and restart client script.

5. The /set_mode service call might need to run twice to display well in rviz.
