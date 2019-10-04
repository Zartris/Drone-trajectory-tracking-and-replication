
## INSTALL
run 
1. sh compiling_scripts/newInstall.sh
2. sh compiling_scripts/ParrotSphinxInstall.sh


[parrot sphinx link](https://developer.parrot.com/docs/sphinx/releasenotes.html)
[general setup](https://forum.developer.parrot.com/t/using-bebop-autonomy-with-sphinx-on-same-machine/6726/5)
[bebop commmands](https://bebop-autonomy.readthedocs.io/en/latest/piloting.html)



insert into .bashrc:
\# ROS PROJECT
source /opt/ros/kinetic/setup.bash
source ~/Code/Drone_MARTIN/mapping/VINS_mono/devel/setup.bash

\# Navigation
source ~/Code/Drone_MARTIN/navigation/devel/setup.sh
\# Bebop_autonomy
source ~/Code/Drone_MARTIN/bebop/devel/setup.bash

\# CUDA
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# Setup

1. Install parrot-sphinx from repos or manually.
2. Install ROS
3. Download and install bebop-autonomy in a ROS workspace
4. In a terminal type “iwconfig” and copy the name that appears first in the network interface IEEE 802.11abgn. In my laptop it changes from time to time, but it’s normally either “eth0” or “wlp2s0”. I think there are ways to set this name permanently, but I didn’t try.
5. In the file “bebop2.drone” (in my computer is placed in /opt/parrot-sphinx/usr/share/sphinx/drones/), paste the name from the previous step in the field “stolen_interface” as follows:
   <stolen_interface>wlp2s0:eth0:192.168.0.5/24</stolen_interface>
   or
   <stolen_interface>eth0:eth0:192.168.0.5/24</stolen_interface>
   depending on the name returned by the command “iwconfig”.
   Notice that I didn’t modified the IP or the port, and I also kept “eth0” in the middle.
   In my case, I keep one of the lines commented because sometimes (normally after the first run of Sphinx) the interface name is modified, so I have to use the other one to avoid getting errors.
6. In the file “bebop_node.launch” change the IP address. I also keep the original one commented for when I need to run the simulator using the WiFi.

1. In one terminal run the following commands:
   sudo systemctl start firmwared.service
   sudo firmwared
2. In a new terminal run the sphinx simulator
   sphinx --datalog /opt/parrot-sphinx/usr/share/sphinx/worlds/outdoor_1.world /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2_local.drone
   Notice that I don’t run the the original file “bebop2.drone”, because I created a new file called “bebop2_local.drone” and modified in this file what I told you before.
3. In a new terminal run the bebop ROS node
   roslaunch bebop_driver bebop_node.launch
4. Finally, to check that it works you can run in a new terminal the following commands to take off tand land he bebop drone
   rostopic pub --once bebop/takeoff std_msgs/Empty
   rostopic pub --once bebop/land std_msgs/Empty

## RUN:
RUN Roscore in first terminal
1. `roscore`

RUN firmware script in second
1. ``sudo systemctl start firmwared.service``
2. ``sudo firmwared``

RUN gazebo in third
1. `sphinx --datalog /opt/parrot-sphinx/usr/share/sphinx/worlds/outdoor_1.world /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2_local.drone`

RUN Bebop ros node in fourth
1. `roslaunch bebop_driver bebop_node.launch`

RUN Image viewer
1. `rosrun image_view image_view image:=/bebop/image_raw`

ROS_NAMESPACE=bebop rosrun image_proc image_proc
rosrun image_view image_view 



**Reset batteri:**
close

1. bebop autonomy
2. firmware
3. sphinx

run `sudo systemctl restart firmwared.service && sudo firmwared `

start `sphinx --datalog /opt/parrot-sphinx/usr/share/sphinx/worlds/outdoor_1.world /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2_local.drone` again

and spam bebop autonomy until it works



# PI

 ssh pi@192.168.42.65

pass: Drone_MARTIN







rosrun kalibr kalibr_calibrate_imu_camera --target dataset/bebop2/april_6x6_A1.yaml --cam bebop2_camera_calib.yaml --imu PiSense_imu_param.yaml --bag dataset/bebop2/calib_A1_1.bag --timeoffset-padding 1 --verbose



# Cam calibration

ros bag to png:

 rosrun image_view extract_images _sec_per_frame:=0.01 _filename_format:="frame%04d.png" image:=/bebop/image_mono 

run calibration 

rosrun camera_model Calibration -w 6 -h 7 -s 40.05 --camera-model pinhole --camera-name bebop-front --opencv --view-results -v -p frame -i img/



