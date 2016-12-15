#!/bin/sh
############
#Need modules: 
#apr-util-1.5.1-x86_64-1.xzm  apr-1.4.6-x86_64-1.xzm  db44-4.4.20-x86_64-3.xzm  git-1.8.4-x86_64-1.xzm  jre-8u101-x86_64-1.xzm  lapack-3.5.0-x86_64-1gv.xzm  neon-0.29.6-x86_64-2.xzm opencv-2.4.7-x86_64-1sl.xzm  subversion-1.7.13-x86_64-1.xzm  sudo-1.8.6p8-x86_64-1.xzm
#Cython-0.23.4-x86_64-1_slack.xzm  eigen3-3.2.4-x86_64-1_slack.xzm  glew-1.9.0-x86_64-1.xzm  numpy-1.9.1-x86_64-1_slack.xzm  pip-7.1.2-x86_64-1_slack.xzm  pysetuptools-18.2-x86_64-3_slack.xzm  python-2.7.5-x86_64-1.xzm  tcl-8.6.1-x86_64-1.xzm  tk-8.6.1-x86_64-1.xzm
#asi nepotrebuju svn, nemam pysetuptools a potrebuju qt5 ###########

echo "RUN AS ROOT."
echo "================================================================================="

git clone https://github.com/TomasBedrnik/ComputerVision.git
git clone https://github.com/TomasBedrnik/rtabmap.git

cd rtabmap/build
cmake ..
make -j2
cd ../..

cd ComputerVision/Kinect2-Grabber-libfreenect2
make
cd ../RealSense-Grabber-librealsense
make
cd ../PCD-Viewer
make
cd ../PCD-Transformer
make
cd ../..

mkdir -p /mnt/sda1/3DVision/rtabmap
cp ComputerVision/Kinect2-Grabber-libfreenect2/grabber-kinect2 /mnt/sda1/3DVision/
cp ComputerVision/RealSense-Grabber-librealsense/grabber-realsense /mnt/sda1/3DVision/
cp ComputerVision/PCD-Viewer/viewer /mnt/sda1/3DVision/
cp ComputerVision/PCD-Transformer/transformer /mnt/sda1/3DVision/
cp rtabmap/bin/* /mnt/sda1/3DVision/rtabmap
cp ComputerVision/Porteus/mapper.sh /mnt/sda1/3DVision/

# mkdir -p dirxzm/home/guest/Desktop/3DVision/rtabmap
# cp ComputerVision/Kinect2-Grabber-libfreenect2/grabber-kinect2 dirxzm/home/guest/Desktop/3DVision/
# cp ComputerVision/RealSense-Grabber-librealsense/grabber-realsense dirxzm/home/guest/Desktop/3DVision/
# cp ComputerVision/PCD-Viewer/viewer dirxzm/home/guest/Desktop/3DVision/
# cp ComputerVision/PCD-Transformer/transformer dirxzm/home/guest/Desktop/3DVision/
# cp rtabmap/bin/* dirxzm/home/guest/Desktop/3DVision/rtabmap
# cp ComputerVision/Porteus/mapper.sh dirxzm/home/guest/Desktop/3DVision/
# chown -R guest:guest dirxzm
# 
# dir2xzm dirxzm /mnt/sda1/porteus/modules/my3DVision.xzm
# activate /mnt/sda1/porteus/modules/my3DVision.xzm
