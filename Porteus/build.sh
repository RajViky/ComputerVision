#!/bin/sh
#############
#SUDO needs to be configured to work withou password
#mount -t vboxsf -o uid=1000,gid=1000 data /mnt/host
#guest ALL=(ALL) NOPASSWD:ALL
#It isnt much safe so you should disable it after compilation
############
#Need modules: 
#apr-util-1.5.1-x86_64-1.xzm  apr-1.4.6-x86_64-1.xzm  db44-4.4.20-x86_64-3.xzm  git-1.8.4-x86_64-1.xzm  jre-8u101-x86_64-1.xzm  lapack-3.5.0-x86_64-1gv.xzm  neon-0.29.6-x86_64-2.xzm opencv-2.4.7-x86_64-1sl.xzm  subversion-1.7.13-x86_64-1.xzm  sudo-1.8.6p8-x86_64-1.xzm
#Cython-0.23.4-x86_64-1_slack.xzm  eigen3-3.2.4-x86_64-1_slack.xzm  glew-1.9.0-x86_64-1.xzm  numpy-1.9.1-x86_64-1_slack.xzm  pip-7.1.2-x86_64-1_slack.xzm  pysetuptools-18.2-x86_64-3_slack.xzm  python-2.7.5-x86_64-1.xzm  tcl-8.6.1-x86_64-1.xzm  tk-8.6.1-x86_64-1.xzm
############

echo "This script will not run without errors after some time."
echo "It is using git repositories, which will change."
echo "You should copy part to compile only one library at once and fix possible errors."
echo "Dont't forget to copy first part with exports of system variables."
read -rsp $'Press any key to continue...\n' -n1 key

export DESTDIR=/opt/playground
export PREFIX=/usr
export LIBDIR=/usr/lib64
export PATH=$DESTDIR/$PREFIX/usr/bin:$PATH
export INCLUDE_PATH=$DESTDIR/usr/include:$INCLUDE_PATH
export LIBRARY_PATH=$DESTDIR/$LIBDIR:$LIBRARY_PATH
export LD_LIBRARY_PATH=$DESTDIR/$LIBDIR:$LD_LIBRARY_PATH

#Pangloin
# git clone https://github.com/stevenlovegrove/Pangolin.git
# cd Pangolin
# mkdir build
# cd build
# cmake -D CMAKE_INSTALL_PREFIX=$PREFIX ..
# make -j4
# sudo make install DESTDIR=$DESTDIR
# cd ../..

#Openni2
git clone https://github.com/occipital/OpenNI2.git
cd OpenNI2
make CFG=Release ALLOW_WARNINGS=1
sudo mkdir -p $DESTDIR/usr/include
sudo mkdir -p $DESTDIR/usr/bin
sudo mkdir -p $DESTDIR/$LIBDIR/pkgconfig
sudo mkdir -p $DESTDIR/etc/udev/rules.d
sudo mkdir -p $DESTDIR/etc/OpenNI2
sudo cp Bin/x64-Release/*.so $DESTDIR/$LIBDIR/
sudo cp -r Bin/x64-Release/OpenNI2 $DESTDIR/$LIBDIR/
sudo cp -r Include/* $DESTDIR/usr/include/
sudo cp Bin/x64-Release/ClosestPointViewer $DESTDIR/usr/bin/
sudo cp Bin/x64-Release/EventBasedRead $DESTDIR/usr/bin/
sudo cp Bin/x64-Release/MultiDepthViewer $DESTDIR/usr/bin/
sudo cp Bin/x64-Release/MultipleStreamRead $DESTDIR/usr/bin/
sudo cp Bin/x64-Release/MWClosestPointApp $DESTDIR/usr/bin/
sudo cp Bin/x64-Release/SimpleRead $DESTDIR/usr/bin/
sudo cp Bin/x64-Release/SimpleViewer $DESTDIR/usr/bin/
sudo cp Packaging/Linux/primesense-usb.rules $DESTDIR/etc/udev/rules.d/557-primesense-usb.rules
sudo cp -r Config/* $DESTDIR/etc/OpenNI2/
export PKGCFGFILE=$DESTDIR/$LIBDIR/pkgconfig/libopenni2.pc
sudo chown guest:users $DESTDIR/$LIBDIR/pkgconfig/
echo "prefix=/usr" >> $PKGCFGFILE
echo "exec_prefix=\${prefix}" >> $PKGCFGFILE
echo "libdir=\${exec_prefix}/lib" >> $PKGCFGFILE
echo "includedir=\${prefix}/include/openni2" >> $PKGCFGFILE
echo "" >> $PKGCFGFILE
echo "Name: OpenNI2" >> $PKGCFGFILE
echo "Description: A general purpose driver for all OpenNI cameras." >> $PKGCFGFILE
echo "Version: 2.2.beta2" >> $PKGCFGFILE
echo "Cflags: -I\${includedir}" >> $PKGCFGFILE
echo "Libs: -L\${libdir} -lOpenNI2" >> $PKGCFGFILE
cd ..

#NEW libUSB
wget http://downloads.sourceforge.net/project/libusb/libusb-1.0/libusb-1.0.20/libusb-1.0.20.tar.bz2 --no-check-certificate
tar xf libusb-1.0.20.tar.bz2
cd libusb-1.0.20
./configure --prefix=$PREFIX --libdir=$LIBDIR
make
sudo make install DESTDIR=$DESTDIR
cd ..

#newer cmake
wget https://cmake.org/files/v3.6/cmake-3.6.2.tar.gz --no-check-certificate
tar xf cmake-3.6.2.tar.gz
cd cmake-3.6.2
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make -j5
sudo make install DESTDIR=$DESTDIR
cd ../..

#libjpeg-turbo
wget http://downloads.sourceforge.net/project/libjpeg-turbo/1.5.1/libjpeg-turbo-1.5.1.tar.gz --no-check-certificate
tar xf libjpeg-turbo-1.5.1.tar.gz
cd libjpeg-turbo-1.5.1
./configure --prefix=$PREFIX --libdir=$LIBDIR --with-jpeg8
make -j5
sudo make install DESTDIR=$DESTDIR
cd ..

#create and activate temporary module
#overwrite existing system things -> easier than without this step
sudo dir2xzm $DESTDIR /opt/prereq0.xzm
sudo /opt/porteus-scripts/activate /opt/prereq0.xzm


#libfreenect2
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$PREFIX \
    -DBUILD_OPENNI2_DRIVER=ON \
    -DBUILD_EXAMPLES=ON \
    -DCMAKE_INSTALL_LIBDIR=$LIBDIR
make -j5
sudo make install DESTDIR=$DESTDIR
cd ../..
sudo cp -r $DESTDIR/usr/lib/* $DESTDIR/$LIBDIR
sudo rm -rf $DESTDIR/usr/lib
sudo sed -i "s|/lib|/lib64|g" $DESTDIR/$LIBDIR/pkgconfig/freenect2.pc


#FLANN
wget http://www.cs.ubc.ca/research/flann/uploads/FLANN/flann-1.8.4-src.zip --no-check-certificate
unzip flann-1.8.4-src.zip 
cd flann-1.8.4-src
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$PREFIX \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_MATLAB_BINDINGS=OFF \
      -DBUILD_PYTHON_BINDINGS=OFF
make -j5
sudo make install DESTDIR=$DESTDIR
cd ../..
sudo cp -r $DESTDIR/usr/lib/* $DESTDIR/$LIBDIR
sudo rm -rf $DESTDIR/usr/lib
sudo sed -i "s|/lib|/lib64|g" $DESTDIR/$LIBDIR/pkgconfig/flann.pc

#HDF5
wget https://support.hdfgroup.org/ftp/HDF5/current/src/hdf5-1.8.17.tar.bz2 --no-check-certificate
tar xf hdf5-1.8.17.tar.bz2
cd hdf5-1.8.17
./configure --prefix=$PREFIX --libdir=$LIBDIR
make -j5
sudo make install DESTDIR=$DESTDIR
cd ..

sudo /opt/porteus-scripts/deactivate $/opt/prereq0.xzm
sudo dir2xzm $DESTDIR /opt/prereq41.xzm
sudo /opt/porteus-scripts/activate /opt/prereq1.xzm

#NetCDF
wget ftp://ftp.unidata.ucar.edu/pub/netcdf/netcdf-4.4.1.tar.gz
tar xf netcdf-4.4.1.tar.gz
cd netcdf-4.4.1
./configure --prefix=$PREFIX --libdir=$LIBDIR --enable-shared --enable-netcdf-4 --enable-dap-netcdf
make -j5
sudo make install DESTDIR=$DESTDIR
cd ..


#NetCDF-cxx
export CPPFLAGS=-I$DESTDIR/usr/include 
export LDFLAGS=-L$DESTDIR/$LIBDIR
wget ftp://ftp.unidata.ucar.edu/pub/netcdf/netcdf-cxx-4.2.tar.gz
tar xf netcdf-cxx-4.2.tar.gz
cd netcdf-cxx-4.2
./configure --prefix=$PREFIX --libdir=$LIBDIR
make -j5
sudo make install DESTDIR=$DESTDIR
cd ..

#GL2PS
wget http://geuz.org/gl2ps/src/gl2ps-1.3.9.tgz --no-check-certificate
tar xf gl2ps-1.3.9.tgz
cd gl2ps-1.3.9-source
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$PREFIX
make -j5
sudo make install DESTDIR=$DESTDIR
cd ../..
sudo cp -r $DESTDIR/usr/lib/* $DESTDIR/$LIBDIR
sudo rm -rf $DESTDIR/usr/lib

#GDAL
wget http://download.osgeo.org/gdal/2.1.1/gdal-2.1.1.tar.xz --no-check-certificate
tar xf gdal-2.1.1.tar.xz
cd gdal-2.1.1
./configure --prefix=$PREFIX --libdir=$LIBDIR --with-netcdf --with-libtiff --with-sqlite3 --with-geotiff \
              --with-mysql --with-python --with-curl --with-hdf5 --with-perl --with-geos \
              --with-png --with-poppler --with-spatialite --with-openjpeg --enable-static=no
make  -j5
sudo make install DESTDIR=$DESTDIR
cd ..

#jsoncpp
git clone https://github.com/open-source-parsers/jsoncpp
cd jsoncpp
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$PREFIX \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_STATIC_LIBS=OFF
make -j5
sudo make install DESTDIR=$DESTDIR
cd ../..
sudo cp -r $DESTDIR/usr/lib/* $DESTDIR/$LIBDIR
sudo rm -rf $DESTDIR/usr/lib
sudo sed -i "s|/lib|/lib64|g" $DESTDIR/$LIBDIR/pkgconfig/jsoncpp.pc


sudo /opt/porteus-scripts/deactivate $/opt/prereq1.xzm
sudo dir2xzm $DESTDIR /opt/prereq2.xzm
sudo /opt/porteus-scripts/activate /opt/prereq2.xzm

#VTK
#git clone https://github.com/Kitware/VTK.git
wget http://www.vtk.org/files/release/7.0/VTK-7.0.0.tar.gz --no-check-certificate
tar xf VTK-7.0.0.tar.gz
cd VTK-7.0.0
wget https://raw.githubusercontent.com/TomasBedrnik/ComputerVision/master/Porteus/patchCMakeLists.txt --no-check-certificate
wget https://raw.githubusercontent.com/TomasBedrnik/ComputerVision/master/Porteus/patchGDAL.txt --no-check-certificate
patch IO/GDAL/vtkGDALVectorReader.cxx < patchGDAL.txt
patch CMakeLists.txt < patchCMakeLists.txt
mkdir build
cd build
cmake .. \
      -DCMAKE_INSTALL_PREFIX=$PREFIX \
      -DBUILD_DOCUMENTATION=OFF \
      -DBUILD_SHARED_LIBS=ON \
      -DBUILD_TESTING=OFF \
      -DVTK_LEGACY_REMOVE=ON \
      -DVTK_USE_CXX11_FEATURES=OFF \
      -DVTK_USE_LARGE_DATA=ON \
      -DVTK_USE_SYSTEM_LIBPROJ4=OFF \
      -DVTK_USE_SYSTEM_XDMF3=OFF \
      -DModule_vtkIOFFMPEG=ON \
      -DModule_vtkIOGDAL=ON \
      -DModule_vtkxdmf3=ON \
      -DModule_vtkIOXdmf3=ON \
      -DVTK_WRAP_JAVA=OFF \
      -DVTK_WRAP_PYTHON=OFF \
      -DVTK_WRAP_TCL=ON \
      -DVTK_USE_SYSTEM_LIBRARIES=ON \
      -DVTK_USE_SYSTEM_GL2PS=OFF \
      -DVTK_USE_SYSTEM_HDF5=ON
make -j5
sudo make install DESTDIR=$DESTDIR
cd ../..  

sudo cp -r $DESTDIR/usr/lib/* $DESTDIR/$LIBDIR
sudo rm -rf $DESTDIR/usr/lib
#sudo sed -i "s|/lib/cmake|/lib64/cmake|g" $DESTDIR/$LIBDIR/cmake/vtk-7.0/*.cmake
#sudo sed -i "s|/usr/lib/|/usr/lib64/|g" $DESTDIR/$LIBDIR/cmake/vtk-7.0/*.cmake


sudo /opt/porteus-scripts/deactivate $/opt/prereq2.xzm
sudo dir2xzm $DESTDIR /opt/prereq3.xzm
sudo /opt/porteus-scripts/activate /opt/prereq3.xzm

#export OPENNI2_INCLUDE=/home/guest/Desktop/openni2/OpenNI-Linux-x64-2.2/Include
#export OPENNI2_REDIST=/home/guest/Desktop/openni2/OpenNI-Linux-x64-2.2/Redist
#cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX
#PCL
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz --no-check-certificate
tar xf pcl-1.8.0.tar.gz
cd pcl-pcl-1.8.0
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DLIB_INSTALL_DIR=$LIBDIR
make -j5
sudo make install DESTDIR=$DESTDIR
cd ../..

#sudo dir2xzm $PREFIX /mnt/sda1/porteus/modules/pcl.xzm
# sudo dir2xzm /opt/libfreenect2 /mnt/sda1/porteus/modules/libfreenect2.xzm

sudo sed -i "s|//usr||g" $DESTDIR/$LIBDIR/pkgconfig/pcl_*.pc


#LibDRM
#wget https://dri.freedesktop.org/libdrm/libdrm-2.4.71.tar.gz --no-check-certificate
#tar xf 
git clone git://anongit.freedesktop.org/mesa/drm
cd drm
./autogen.sh
./configure --prefix=$PREFIX --libdir=$LIBDIR --enable-udev
make
sudo make install DESTDIR=$DESTDIR
cd ..
#

#dri3proto
wget https://www.x.org/releases/individual/proto/dri3proto-1.0.tar.bz2 --no-check-certificate
tar xf dri3proto-1.0.tar.bz2
cd dri3proto-1.0
./configure --prefix=$PREFIX --libdir=$LIBDIR
sudo make install DESTDIR=$DESTDIR
cd ..

#presentproto
wget  http://xorg.freedesktop.org/releases/individual/proto/presentproto-1.0.tar.bz2 --no-check-certificate
tar xf presentproto-1.0.tar.bz2
cd presentproto-1.0
./configure --prefix=$PREFIX --libdir=$LIBDIR
sudo make install DESTDIR=$DESTDIR
cd ..


#xcb-proto
export pkgname=xcb-proto
export pkgver=1.12
export url="http://xcb.freedesktop.org/"
wget $url/dist/$pkgname-$pkgver.tar.bz2 --no-check-certificate
tar xf $pkgname-$pkgver.tar.bz2
cd $pkgname-$pkgver
./configure --prefix=$PREFIX --libdir=$LIBDIR
make
sudo make install DESTDIR=$DESTDIR
cd ..

sudo /opt/porteus-scripts/deactivate $/opt/prereq3.xzm
sudo dir2xzm $DESTDIR /opt/prereq4.xzm
sudo /opt/porteus-scripts/activate /opt/prereq4.xzm

#libxcb
#export PKG_CONFIG_PATH=$DESTDIR/$LIBDIR/pkgconfig
export pkgname=libxcb
export pkgver=1.12
export url="http://xcb.freedesktop.org/"
wget $url/dist/$pkgname-$pkgver.tar.bz2 --no-check-certificate
tar xf $pkgname-$pkgver.tar.bz2
cd $pkgname-$pkgver
./configure --prefix=$PREFIX --libdir=$LIBDIR \
    --enable-xinput \
    --enable-xkb \
    --disable-static
make
sudo make install DESTDIR=$DESTDIR
cd ..

#xproto - maybe I will need this in newer verwion

#libxshmfence
export pkgname=libxshmfence
export pkgver=1.2
export url="http://xorg.freedesktop.org/"
wget $url/releases/individual/lib/$pkgname-$pkgver.tar.bz2 --no-check-certificate
tar xf $pkgname-$pkgver.tar.bz2
cd $pkgname-$pkgver
./configure --prefix=$PREFIX --libdir=$LIBDIR
make
sudo make install DESTDIR=$DESTDIR
cd ..
sudo rm $DESTDIR/$LIBDIR/$pkgname.a


sudo /opt/porteus-scripts/deactivate /opt/prereq4.xzm
sudo dir2xzm $DESTDIR /opt/prereq5.xzm
sudo /opt/porteus-scripts/activate /opt/prereq5.xzm


wget https://mesa.freedesktop.org/archive/12.0.3/mesa-12.0.3.tar.xz --no-check-certificate
tar xf mesa-12.0.3.tar.xz
cd mesa-12.0.3
./configure --prefix=$PREFIX --libdir=$LIBDIR --disable-gallium-llvm \
               --with-gallium-drivers=i915,ilo,svga,swrast,virgl \
               --with-dri-drivers=i915,i965,swrast \
               --with-egl-platforms=x11,drm \
               --disable-vulkan-icd-full-driver-path \
               --with-sha1=libnettle \
               --enable-texture-float \
               --enable-dri3 \
               --enable-osmesa \
               --enable-xa \
               --enable-gbm \
               --enable-nine \
               --enable-xvmc \
               --enable-glx-tlscd
make -j5
sudo make install DESTDIR=$DESTDIR
cd ..


sudo /opt/porteus-scripts/deactivate /opt/prereq5.xzm
sudo dir2xzm $DESTDIR /mnt/sda1/porteus/modules/mesa.xzm
sudo /opt/porteus-scripts/activate /mnt/sda1/porteus/modules/mesa.xzm

# export DESTDIR=/opt/opencv2
# #OpenCV 2.4.13
# wget http://github.com/Itseez/opencv/archive/2.4.13.zip --no-check-certificate
# unzip 2.4.13.zip
# cd opencv-2.4.13
# mkdir release
# cd release
# cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$PREFIX cmake -DENABLE_PRECOMPILED_HEADERS=OFF ..
# make -j5
# sudo make install DESTDIR=$DESTDIR
# cd ../..
# sudo dir2xzm $DESTDIR /mnt/sda1/porteus/modules/opencv2.xzm
# sudo /opt/porteus-scripts/activate /mnt/sda1/porteus/modules/opencv2.xzm


export DESTDIR=/opt/rtabmap
export PREFIX=/usr
export LIBDIR=/usr/lib64
export PATH=$DESTDIR/$PREFIX/usr/bin:$PATH
export INCLUDE_PATH=/usr/include/eigen3:$INCLUDE_PATH
export LIBRARY_PATH=$DESTDIR/$LIBDIR:$LIBRARY_PATH
export LD_LIBRARY_PATH=$DESTDIR/$LIBDIR:$LD_LIBRARY_PATH

git clone https://github.com/introlab/rtabmap.git rtabmap
cd rtabmap/build
cmake .. -DCMAKE_INSTALL_PREFIX=$PREFIX
make -j5
sudo make install DESTDIR=$DESTDIR


cmake .. \
      -DCMAKE_INSTALL_PREFIX=$PREFIX \
      -DBUILD_DOCUMENTATION=OFF \
      -DBUILD_SHARED_LIBS=ON \
      -DBUILD_TESTING=OFF \
      -DVTK_LEGACY_REMOVE=ON \
      -DVTK_USE_CXX11_FEATURES=OFF \
      -DVTK_USE_LARGE_DATA=ON \
      -DVTK_USE_SYSTEM_LIBPROJ4=OFF \
      -DVTK_USE_SYSTEM_XDMF3=OFF \
      -DModule_vtkIOFFMPEG=ON \
      -DModule_vtkIOGDAL=ON \
      -DModule_vtkxdmf3=ON \
      -DModule_vtkIOXdmf3=ON \
      -DVTK_WRAP_JAVA=OFF \
      -DVTK_WRAP_PYTHON=OFF \
      -DVTK_WRAP_TCL=ON \
      -DVTK_USE_SYSTEM_LIBRARIES=ON \
      -DVTK_USE_SYSTEM_GL2PS=OFF \
      -DVTK_USE_SYSTEM_HDF5=ON \
      -DModule_vtkGUISupportQtOpenGL:BOOL=ON \
      -DModule_vtkGUISupportQt:BOOL=ON \
      -DVTK_Group_Qt:BOOL=ON
make -j5
sudo make install DESTDIR=$DESTDIR
sudo dir2xzm $DESTDIR /mnt/sda1/porteus/modules/mesa.xzm
#VTK
cmake .. \
      -DCMAKE_INSTALL_PREFIX=$PREFIX/usr \
      -DBUILD_DOCUMENTATION=OFF \
      -DBUILD_SHARED_LIBS=ON \
      -DBUILD_TESTING=OFF \
      -DVTK_LEGACY_REMOVE=ON \
      -DVTK_USE_CXX11_FEATURES=OFF \
      -DVTK_USE_LARGE_DATA=ON \
      -DVTK_USE_SYSTEM_LIBPROJ4=OFF \
      -DVTK_USE_SYSTEM_XDMF3=OFF \
      -DModule_vtkIOFFMPEG=ON \
      -DModule_vtkIOGDAL=ON \
      -DModule_vtkxdmf3=ON \
      -DModule_vtkIOXdmf3=ON \
      -DVTK_WRAP_JAVA=OFF \
      -DVTK_WRAP_PYTHON=ON \
      -DVTK_WRAP_TCL=ON \
      -DVTK_USE_SYSTEM_LIBRARIES=ON \
      -DVTK_USE_SYSTEM_GL2PS=OFF \
      -DVTK_USE_SYSTEM_HDF5=ON \
      -DQT_QMAKE_EXECUTABLE:PATH=/usr/bin/qmake \
      -DVTK_Group_Qt:BOOL=ON \
      -DVTK_USE_FFMPEG_ENCODER:BOOL=ON  \
      -DModule_vtkGUISupportQtWebkit:BOOL=ON \
      -DModule_vtkRenderingOpenGL2:BOOL=ON \
      -DModule_vtkFiltersParallelFlowPaths:BOOL=ON \
      -DModule_vtkParallelMPI:BOOL=ON \
      -DModule_vtkFiltersParallelGeometry:BOOL=ON \
      -DModule_vtkFiltersParallelMPI:BOOL=ON \
      -DModule_vtkFiltersParallelStatistics:BOOL=ON \
      -DModule_vtkFiltersPython:BOOL=ON \
      -DModule_vtkFiltersReebGraph:BOOL=ON \
      -DModule_vtkGUISupportQtOpenGL:BOOL=ON \
      -DModule_vtkWrappingPythonCore:BOOL=OFF