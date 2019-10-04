# Drone_MARTIN

[A nice intro]

## General info

The drone used in this project is [Bebop 2 (parrot)](https://www.parrot.com/eu/drones/parrot-bebop-2-fpv#technicals) containing a 14 mega-pixels camera with fish-eye lens for video feed up to HD 1080p.



## Packages used:

Slam package = LCSD_SLAM : https://github.com/sunghoon031/LCSD_SLAM 

Controlling and getting the videofeed = bebop_autonomy : https://github.com/AutonomyLab/bebop_autonomy



## INSTALL

First of all if you see a `~/Code/Drone_MARTIN/` somewhere, then it is my `[YOUR_PATH]` so change this to the path into your folder.



Alright for easy first-time run, delete folder LCSD_SLAM and run `newInstall.sh`.
This will install install all dependencies for LCSD including Pangolin.



Then cd into LCSD_SLAM after install, and change `build.sh` line 1:

1. from `cd ~/LCSD_SLAM` to `cd [YOUR_PATH]/LCSD_SLAM/`





and line 40:

40. from: `grep -q -F 'export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/LCSD_SLAM/Examples/ROS' ~/.bashrc || echo 'export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/LCSD_SLAM/Examples/ROS' >> ~/.bashrc` 

    to : `grep -q -F 'export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:[YOUR_PATH]/LCSD_SLAM/Examples/ROS' ~/.bashrc || echo 'export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:[YOUR_PATH]/LCSD_SLAM/Examples/ROS' >> ~/.bashrc` 



## SETUP BASHRC

Make the export happen for every new terminal:

open with command:	`gedit ~/.bashrc`

and insert at end :

````bash
source ~/Code/Drone_MARTIN/LCSD_SLAM/ORB_SLAM2/Examples/ROS/ORB_SLAM2/build/devel/setup.bash
source ~/Code/Drone_MARTIN/LCSD_SLAM/DSO_ROS/catkin_ws/devel/setup.bash
export DSO_PATH=~/Code/Drone_MARTIN/LCSD_SLAM/DSO
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Code/Drone_MARTIN/LCSD_SLAM/
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Code/Drone_MARTIN/LCSD_SLAM/ORB_SLAM2/Examples/ROS
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Code/Drone_MARTIN/LCSD_SLAM/DSO_ROS/catkin_ws/src/dso_ros
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Code/Drone_MARTIN/LCSD_SLAM/Examples/ROS
````



## Test LCSD_SLAM

Before starting we have to change some parameters:

````bash
cd [YOUR_PATH]/LCSD_SLAM/ORB_SLAM2/Examples/Monocular/TUMmonoVO_yaml/

Find the file you want to run:
	monoVO_ORB_[VO/SLAM]_full_[SEQUENCE_NUMBER].yaml
    Change Gui parameter to 1 for visuals and 0 for none.
    Rest should work fine.

Then go to:
cd [YOUR_PATH]/LCSD_SLAM/DSO_ROS/catkin_ws/src/dso_ros/launch/

Find the simular file from before:
	monoVO_seong_[SEQUENCE_NUMBER].launch
	Change:
	display_GUI = true or false
	playback_speed = 0.001<x<=10, where 1 is real-time.
	
	Then change the path to the dataset:
	<param name="image_file_path" type="string" value="[YOUR_PATH_TO_DATASET]/TUM_monoVO/sequence_01/images.zip"/>
        <param name="calib_file_path" type="string" value="[YOUR_PATH_TO_DATASET]/TUM_monoVO/sequence_01/camera.txt"/>
        <param name="vignette_file_path" type="string" value="[YOUR_PATH_TO_DATASET]/TUM_monoVO/sequence_01/vignette.png"/>
        <param name="gamma_file_path" type="string" value="[YOUR_PATH_TO_DATASET]/TUM_monoVO/sequence_01/pcalib.txt"/>
        <param name="stats_file_path" type="string" value=" [YOUR_PATH]/LCSD_SLAM/DSO_ROS/catkin_ws/src/dso_ros/trackingStats.txt"/>
	
	
My [Your_PATH_TO_DATASET] is "/home/zartris/Code/datasets/", where i find using ~ not enough here and have to write the full path.
````



**To run:**

&nbsp;&nbsp;&nbsp;&nbsp;(i) In the first terminal, run `roscore`

&nbsp;&nbsp;&nbsp;&nbsp;(ii) In the second terminal, run

```
cd [YOUR_PATH]/LCSD_SLAM/ORB_SLAM2

// For the EuRoC MAV dataset:
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt \
Examples/Monocular/EuRoC_seong_[VO/SLAM]_cam[0/1].yaml

// For the TUM monoVO dataset:
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt \
Examples/Monocular/TUMmonoVO_yaml/monoVO_ORB_[VO/SLAM]_full_[SEQUENCE_NUMBER].yaml 
```

&nbsp;&nbsp;&nbsp;&nbsp;(iii) In the third terminal, run

```
cd [YOUR_PATH]/DSO_ROS/catkin_ws
source devel/setup.bash

// For the EuRoC MAV dataset (see launch files in LCSD_SLAM/DSO_ROS/catkin_ws/src/dso_ros/launch):
roslaunch dso_ros EuRoC_seong_[SEQUENCE_NAME]_cam[0/1].launch 

// For the TUM monoVO dataset (see launch files in LCSD_SLAM/DSO_ROS/catkin_ws/src/dso_ros/launch):
roslaunch dso_ros monoVO_seong_[SEQUENCE_NUMBER].launch
```

The keyframe trajectory will be saved as `KeyFrameTrajectory_seong.txt` in `ORB_SLAM2` folder, and additional tracking statistics will be saved as `trackingStats.txt` in the path you set (`stats_file_path`) in [Step 3](https://github.com/sunghoon031/LCSD-SLAM/blob/master/README.md#3-set-paths).

