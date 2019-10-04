cd ~/Code/Drone_MARTIN/

## CREATING DIRS
printf "\n##################\n"
printf "Creating dirs"
printf "\n##################\n\n"
mkdir bebop bebop/src
mkdir mapping
mkdir mapping/VI-MEAN mapping/VI-MEAN/src
mkdir mapping/VINS_mono mapping/VINS_mono/src

printf "\n##################\n"
printf "Creating BEBOP_AUTONOMY"
printf "\n##################\n\n"
cd bebop
sudo apt-get install build-essential python-rosdep python-catkin-tools
sudo apt-get install libsuitesparse-dev libeigen3-dev libboost-all-dev libopencv-dev
catkin init
git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy
rosdep update
rosdep install --from-paths src -i
catkin build
cd ..


printf "\n##################\n"
printf "Creating VINS_mono"
printf "\n##################\n\n"
# Drone_MARTIN/mapping/VINS_mono
cd workspace/src/VINS_mono
sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-tf ros-kinetic-message-filters ros-kinetic-image-transport
printf "\n INSTALLING DEP CERES SOLVER:\n"
mkdir thirdparty
# Drone_MARTIN/mapping/VINS_mono/thirdparty
cd thirdparty
git clone https://ceres-solver.googlesource.com/ceres-solver
###### GET DEP:
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse and CXSparse (optional)
# - If you want to build Ceres as a *static* library (the default)
#   you can use the SuiteSparse package in the main Ubuntu package
#   repository:
sudo apt-get install libsuitesparse-dev
sudo apt-get update

###### INSTALL:
mkdir ceres-bin
# Drone_MARTIN/mapping/VINS_mono/thirdparty/ceres-bin
cd ceres-bin
cmake ../ceres-solver
make -j3
make test
make install

## GO TO MAPPING FOLDER:
# Drone_MARTIN/mapping/VINS_mono/src
cd ../../src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
# Drone_MARTIN/mapping/VINS_mono
cd ../
catkin_make
source devel/setup.bash
# Drone_MARTIN/mapping
cd ../
printf "\n##################\n"
printf "Creating VI_MEAN"
printf "\n##################\n\n"

cd VI-MEAN/src
git clone https://github.com/HKUST-Aerial-Robotics/VI-MEAN.git
#Drone_MARTIN/mapping
cd ../../
printf "\n NOT INSTALLED FOLLOW GUIDELINES IN https://github.com/HKUST-Aerial-Robotics/VI-MEAN n\n"


