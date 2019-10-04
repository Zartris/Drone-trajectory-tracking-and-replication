# RUN Roscore in first terminal
gnome-terminal --window-with-profile=scripting -e roscore
sleep 3
# RUN firmware script in second
gnome-terminal --window-with-profile=scripting -e sh /home/zartris/Code/Drone_MARTIN/compiling_scripts/testingScripts/firmware_term.sh

sleep 5
# RUN gazebo in third
gnome-terminal --window-with-profile=scripting -e sphinx --datalog /opt/parrot-sphinx/usr/share/sphinx/worlds/outdoor_1.world /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2_local.drone
# RUN Bebop ros node in fourth
gnome-terminal --window-with-profile=scripting -e roslaunch bebop_driver bebop_node.launch


