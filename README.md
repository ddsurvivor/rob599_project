# ROB599 homework 3
---
## WeitongLiang
### 933955080
### liangw@oregonstate.edu
---

## Introduction

This project mainly realizes a logistics robot that automatically carries goods, which can neatly palletize the scattered goods into the warehouse.

Introduction video:

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
