#!/usr/bin/sh
#############
#SUDO needs to be configured to work withou password
#mount -t vboxsf -o uid=1000,gid=1000 data /mnt/host
#guest ALL=(ALL) NOPASSWD:ALL
#It isnt much safe so you should disable it after compilation
############
#Needs subversion (with dependencies) and git modules
#JDK
############

#export PREFIX=/opt/playground2

#export PATH=$PREFIX/bin:$PATH
#export LIBRARY_PATH=$PREFIX/lib:$PREFIX/usr/lib:$LIBRARY_PATH
#export LD_LIBRARY_PATH=$PREFIX/lib:$PREFIX/usr/lib:$LD_LIBRARY_PATH

wget http://com.occipital.openni.s3.amazonaws.com/OpenNI-Linux-x64-2.2.0.33.tar.bz2  --no-check-certificate
tar -xjf OpenNI-Linux-x64-2.2.0.33.tar.bz2 
cd OpenNI-Linux-x64-2.2
sudo ./install.sh
#####################
#UÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁÁ
#####################
#IT IS NIGHTMARE TO INSTALL THIS, USE BINARY PACKAGE INSTEAD

#sudo mkdir $PREFIX
#Openni
#git clone https://github.com/occipital/OpenNI2.git
#cd OpenNI2
#make

#install is nightmare - they dont stick to convetsions

#WTF
#sudo mkdir $PREFIX/etc
#sudo mkdir $PREFIX/etc/OpenNI2
#sudo mkdir $PREFIX/etc/OpenNI2/OpenNI2/
#sudo mkdir $PREFIX/etc/OpenNI2/OpenNI2/Drivers
#sudo mkdir $PREFIX/etc/udev
#sudo mkdir $PREFIX/etc/udev/rules.d
#sudo cp Config/OpenNI.ini $PREFIX/etc/OpenNI2
#sudo cp Config/OpenNI2/Drivers/* $PREFIX/etc/OpenNI2/OpenNI2/Drivers
#sudo cp Packaging/Linux/primesense-usb.rules $PREFIX/etc/udev

#sudo mkdir $PREFIX/usr
#sudo mkdir $PREFIX/usr/bin
#sudo mkdir $PREFIX/usr/include
#sudo mkdir $PREFIX/usr/lib


#mkdir $PREFIX/../etc/ld.so.conf.d/


 # install -d -m755 "${pkgdir}/etc/ld.so.conf.d"
 # echo /usr/lib/OpenNI2/Drivers > "${pkgdir}/etc/ld.so.conf.d/openni2.conf"

  #cp -r "${srcdir}/OpenNI2-${_pkgver}/Config" "${pkgdir}/etc/OpenNI2"

 # install -d -m755 "${pkgdir}/usr/"{bin,lib/OpenNI2/Drivers,share/doc/OpenNI2}

 # cp -r "${srcdir}/OpenNI2-${_pkgver}/Include" "${pkgdir}/usr/include"

 # cd "${srcdir}/OpenNI2-${_pkgver}/Bin/${_platform}-${_cfg}"
 # install *.so "${pkgdir}/usr/lib"
 # install ClosestPointViewer EventBasedRead MultiDepthViewer MultipleStreamRead MWClosestPointApp NiViewer PS1080Console PSLinkConsole SimpleRead SimpleViewer "${pkgdir}/usr/bin"

#  cd "${srcdir}/OpenNI2-${_pkgver}/Bin/${_platform}-${_cfg}/OpenNI2/Drivers"
#  install *.so "${pkgdir}/usr/lib/OpenNI2/Drivers"

#  cd "${srcdir}/OpenNI2-${_pkgver}/Source/Documentation/html"
#  install * "${pkgdir}/usr/share/doc/OpenNI2"

# cd "${srcdir}/OpenNI2-${_pkgver}/Packaging/Linux"
 # sed -i s%/etc/udev/rules.d/%${pkgdir}/etc/udev/rules.d/% install.sh
 # install -d -m755 "${pkgdir}/etc/udev/rules.d"
#  sh install.sh

#  install -d -m755 "${pkgdir}/usr/lib/pkgconfig"
#  cp "${srcdir}/libopenni2.pc" "${pkgdir}/usr/lib/pkgconfig/"


#NEW libUSB
export PREFIX=/opt/libusb/usr
wget http://downloads.sourceforge.net/project/libusb/libusb-1.0/libusb-1.0.20/libusb-1.0.20.tar.bz2 --no-check-certificate
tar -xvf libusb-1.0.20.tar.bz2
cd libusb-1.0.20
./configure --prefix=$PREFIX
make
sudo make install
sudo mkdir $PREFIX/lib64
sudo ln -s ../lib/libusb-1.0.so $PREFIX/lib64/libusb-1.0.so
sudo ln -s ../lib/libusb-1.0.so.0 $PREFIX/lib64/libusb-1.0.so.0
sudo ln -s ../lib/libusb-1.0.so.0.1.0 $PREFIX/lib64/libusb-1.0.so.0.1.0
sudo mkdir $PREFIX/lib64/pkgconfig
sudo ln -s ../../lib/pkgconfig/libusb-1.0.pc $PREFIX/lib64/pkgconfig/libusb-1.0.pc

#libfreenect
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect
mkdir build
cd build
cmake -L -DCMAKE_INSTALL_PREFIX=/opt/libfreenect/usr .. # -L lists all the project options
make
make sudo make install
cd ../..

#newer cmake!!!
wget https://cmake.org/files/v3.6/cmake-3.6.2.tar.gz --no-check-certificate
tar -xzf cmake-3.6.2.tar.gz
cd cmake-3.6.2
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/opt/cmake/usr ..
make
sudo make install
cd ../..

#libjpeg-turbo
wget http://downloads.sourceforge.net/project/libjpeg-turbo/1.5.1/libjpeg-turbo-1.5.1.tar.gz --no-check-certificate
tar -xvf libjpeg-turbo-1.5.1.tar.gz
cd libjpeg-turbo-1.5.1
./configure --prefix=/opt/libjpeg-turbo/usr --with-jpeg8
make
sudo make install
cd ..
sudo dir2xzm /opt/libjpeg-turbo /mnt/sda1/porteus/modules/libjpeg-turbo.xzm

#libfreenect2
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/libfreenect2/usr -DBUILD_OPENNI2_DRIVER=ON -DBUILD_EXAMPLES=ON -DOpenNI2_LIBRARY=/home/guest/Desktop/openni2/OpenNI-Linux-x64-2.2/Redist/ -DOpenNI2_INCLUDE_DIRS=/home/guest/Desktop/openni2/OpenNI-Linux-x64-2.2/Include/
make
sudo make install
cd ../..
sudo dir2xzm /opt/libfreenect2 /mnt/sda1/porteus/modules/libfreenect2.xzm


