# RUN Roscore in first terminal
`roscore`

# RUN firmware script in second

1. ``sudo systemctl start firmwared.service``
2. ``sudo firmwared``

# RUN gazebo in third
1. `sphinx --datalog /opt/parrot-sphinx/usr/share/sphinx/worlds/outdoor_1.world /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2_local.drone`

# RUN Bebop ros node in fourth
1. `roslaunch bebop_driver bebop_node.launch`

# RUN Image viewer

1. `rosrun image_view image_view image:=/bebop/image_raw`

