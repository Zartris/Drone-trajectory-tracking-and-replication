printf "Installing dependencies\n"
sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-tf ros-kinetic-message-filters ros-kinetic-image-transport build-essential python-rosdep python-catkin-tools libsuitesparse-dev libeigen3-dev libboost-all-dev libopencv-dev cmake libgoogle-glog-dev libatlas-base-dev libeigen-dev libsuitesparse-dev cmake g++ git google-mock libboost-all-dev libcairo2-dev libgflags-dev libgoogle-glog-dev liblua5.2-dev libsuitesparse-dev ninja-build python sphinx stow ros-kinetic-parrot-arsdk -y

printf "Fetching ceres solver...\n"
rm -rf ../thirdparty
mkdir ../thirdparty
cd ../thirdparty
git clone https://ceres-solver.googlesource.com/ceres-solver

printf "Injecting ceres-solver into workspace\n"
rm -rf ../workspace/thirdparty
mkdir ../workspace/thirdparty
cd ../workspace/thirdparty
mkdir ceres-bin
cd ceres-bin
cmake ../../../thirdparty/ceres-solver
make -j3
make test
make install
