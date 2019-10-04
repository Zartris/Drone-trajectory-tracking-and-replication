cd ~/Code/Drone_MARTIN/

printf "\n##################\n"
printf "Install dependencies"
printf "\n##################\n\n"
sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-tf ros-kinetic-message-filters ros-kinetic-image-transport -y
sudo apt-get install build-essential python-rosdep python-catkin-tools -y
sudo apt-get install libsuitesparse-dev libeigen3-dev libboost-all-dev libopencv-dev -y

printf "\n##################\n"
printf "Creating BEBOP_AUTONOMY"
printf "\n##################\n\n"
cd bebop
catkin build
cd ..


printf "\n##################\n"
printf "Creating VINS_mono"
printf "\n##################\n\n"
# Drone_MARTIN/mapping/VINS_mono
cd mapping/VINS_mono
printf "\n INSTALLING DEP CERES SOLVER:\n"

rm -rf thirdparty

mkdir thirdparty
# Drone_MARTIN/mapping/VINS_mono/thirdparty
cd thirdparty

git clone https://ceres-solver.googlesource.com/ceres-solver
###### GET DEP:
# CMake
sudo apt-get install cmake -y
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev -y
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev -y
# Eigen3
sudo apt-get install libeigen3-dev -y
# SuiteSparse and CXSparse (optional)
# - If you want to build Ceres as a *static* library (the default)
#   you can use the SuiteSparse package in the main Ubuntu package
#   repository:
sudo apt-get install libsuitesparse-dev -y
sudo apt-get update -y

###### INSTALL:
mkdir ceres-bin
# Drone_MARTIN/mapping/VINS_mono/thirdparty/ceres-bin
cd ceres-bin
cmake ../ceres-solver
make -j3
make test
make install

## GO TO MAPPING FOLDER:
# Drone_MARTIN/mapping/VINS_mono/
cd ../../
catkin_make
source devel/setup.bash
# Drone_MARTIN/
cd ../../

printf "\n##################\n"
printf "Creating msg"
printf "\n##################\n\n"
# Drone_MARTIN/msg
cd msg
catkin_make
catkin_make install
source devel/setup.bash
cd ..

printf "\n##################\n"
printf "Creating Navigation"
printf "\n##################\n\n"
cd navigation
catkin_make
cd ..

printf "\n##################\n"
printf "Creating Pathing"
printf "\n##################\n\n"
cd MSG
catkin_make
cd ..

printf "\n##################\n"
printf "Creating VI_MEAN"
printf "\n##################\n\n"
printf "\n NOT INSTALLED FOLLOW GUIDELINES IN https://github.com/HKUST-Aerial-Robotics/VI-MEAN n\n"


