printf "\n##################\n"
printf "Pangolin.git"
printf "\n##################\n\n"
sudo apt-get install libglew-dev cmake libpython2.7-dev ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev libdc1394-22-dev libraw1394-dev libjpeg-dev libpng12-dev libtiff5-dev libopenexr-dev libsuitesparse-dev

rm -rf Pangolin
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .

cd ~/Code/Drone_MARTIN/LCSD_SLAM/

sudo apt-get install zlib1g-dev
cd DSO/thirdparty
tar -zxvf libzip-1.1.1.tar.gz
cd libzip-1.1.1/
./configure
make
sudo make install
sudo cp lib/zipconf.h /usr/local/include/zipconf.h
