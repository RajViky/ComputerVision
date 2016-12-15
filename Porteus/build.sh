#!/bin/sh
############
#Need modules: 
#apr-util-1.5.1-x86_64-1.xzm  apr-1.4.6-x86_64-1.xzm  db44-4.4.20-x86_64-3.xzm  git-1.8.4-x86_64-1.xzm  jre-8u101-x86_64-1.xzm  lapack-3.5.0-x86_64-1gv.xzm  neon-0.29.6-x86_64-2.xzm opencv-2.4.7-x86_64-1sl.xzm  subversion-1.7.13-x86_64-1.xzm  sudo-1.8.6p8-x86_64-1.xzm
#Cython-0.23.4-x86_64-1_slack.xzm  eigen3-3.2.4-x86_64-1_slack.xzm  glew-1.9.0-x86_64-1.xzm  numpy-1.9.1-x86_64-1_slack.xzm  pip-7.1.2-x86_64-1_slack.xzm  pysetuptools-18.2-x86_64-3_slack.xzm  python-2.7.5-x86_64-1.xzm  tcl-8.6.1-x86_64-1.xzm  tk-8.6.1-x86_64-1.xzm
#asi nepotrebuju svn, nemam pysetuptools a potrebuju qt5 ###########

echo "This script will not run without errors after some time."
echo "It is using git repositories, which will change."
echo "You should copy part to compile only one library at once and fix possible errors."
echo "Dont't forget to copy first part with exports of system variables."
echo "RUN AS ROOT."
echo "================================================================================="

read -rsp $'Press any key to continue...\n' -n1 key

export DESTDIR=/opt/playground
export PREFIX=/usr
export LIBDIR=/usr/lib64
export PATH=$DESTDIR/$PREFIX/usr/bin:$PATH
export INCLUDE_PATH=$DESTDIR/usr/include:$INCLUDE_PATH
export LIBRARY_PATH=$DESTDIR/$LIBDIR:$LIBRARY_PATH
export LD_LIBRARY_PATH=$DESTDIR/$LIBDIR:$LD_LIBRARY_PATH

read -p "Download sources? yes/no:  " yn
case $yn in
    [Yy]* ) echo "Downloading..."
        mkdir sources
        cd sources
        #openni2
        git clone https://github.com/occipital/OpenNI2.git
        #freenect2
        git clone https://github.com/OpenKinect/libfreenect2.git
        #FLANN
        wget http://www.cs.ubc.ca/research/flann/uploads/FLANN/flann-1.8.4-src.zip --no-check-certificate
        #GDAL
        wget http://download.osgeo.org/gdal/2.1.1/gdal-2.1.1.tar.xz --no-check-certificate
        #NETCDFcxx
        wget ftp://ftp.unidata.ucar.edu/pub/netcdf/netcdf-cxx-4.2.tar.gz
        #VTK
        wget http://www.vtk.org/files/release/7.0/VTK-7.0.0.tar.gz --no-check-certificate
        #PCL
        wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz --no-check-certificate
        #librealsense
        git clone https://github.com/IntelRealSense/librealsense.git
        #opencv
        wget https://github.com/Itseez/opencv/archive/3.1.0.zip --no-check-certificate
        wget https://github.com/Itseez/opencv_contrib/archive/3.1.0.tar.gz --no-check-certificate
        #DONE
        cd ..
        echo "Done.";;
    [Nn]* ) echo "Skipping...";;
    * ) echo "Please answer yes or no.";;
esac

echo "Openni2" #No dep
read -p "Build? yes/no:  " yn
case $yn in
    [Yy]* ) echo "building..."
        cp -r sources/OpenNI2 ./
        cd OpenNI2
        make CFG=Release ALLOW_WARNINGS=1
        mkdir -p $DESTDIR/usr/include
        mkdir -p $DESTDIR/usr/bin
        mkdir -p $DESTDIR/$LIBDIR/pkgconfig
        mkdir -p $DESTDIR/etc/udev/rules.d
        mkdir -p $DESTDIR/etc/OpenNI2
        cp Bin/x64-Release/*.so $DESTDIR/$LIBDIR/
        cp -r Bin/x64-Release/OpenNI2 $DESTDIR/$LIBDIR/
        cp -r Include/* $DESTDIR/usr/include/
        cp Packaging/Linux/primesense-usb.rules $DESTDIR/etc/udev/rules.d/557-primesense-usb.rules
        cp -r Config/* $DESTDIR/etc/OpenNI2/
        export PKGCFGFILE=$DESTDIR/$LIBDIR/pkgconfig/libopenni2.pc
        chown guest:users $DESTDIR/$LIBDIR/pkgconfig/
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
        cd ..;;
    [Nn]* ) echo "Skipping...";;
        * ) echo "Please answer yes or no.";;
esac

#libfreenect2
echo "libfreenect2" #dep: only openni
read -p "Build? yes/no:  " yn
case $yn in
    [Yy]* ) echo "building..."
        cp -r sources/libfreenect2 ./
        cd libfreenect2
        mkdir build
        cd build
        cmake .. -DCMAKE_INSTALL_PREFIX=$PREFIX \
            -DCMAKE_PREFIX_PATH=$DESTDIR$PREFIX \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_OPENNI2_DRIVER=ON \
            -DENABLE_CXX11=ON \
            -DBUILD_EXAMPLES=ON \
            -DCMAKE_INSTALL_LIBDIR=$LIBDIR
        make -j5
        make install DESTDIR=$DESTDIR       
        
        mkdir -p $DESTDIR/etc/udev/rules.d/
        cp ../platform/linux/udev/90-kinect2.rules $DESTDIR/etc/udev/rules.d/

        cd ../..
        cp -r $DESTDIR/usr/lib/* $DESTDIR/$LIBDIR
        rm -rf $DESTDIR/usr/lib
        sed -i "s|/lib|/lib64|g" $DESTDIR/$LIBDIR/pkgconfig/freenect2.pc;;
    [Nn]* ) echo "Skipping...";;
    * ) echo "Please answer yes or no.";;
esac


echo "librealsense" #dep: not tested
read -p "Build? yes/no:  " yn
case $yn in
    [Yy]* ) echo "building..."
        cp -r sources/librealsense ./
        cd librealsense
        mkdir build && cd build
        cmake .. -DCMAKE_INSTALL_PREFIX=$PREFIX \
            -DCMAKE_PREFIX_PATH=$DESTDIR$PREFIX \
            -DLIB_INSTALL_DIR=$LIBDIR
        make -j4
        make install DESTDIR=$DESTDIR
        #build kernel module
        cd ../scripts
        wget https://raw.githubusercontent.com/TomasBedrnik/ComputerVision/master/Porteus/patch-porteus.sh --no-check-certificate
        chmod +x patch-porteus.sh
        ./patch-porteus.sh
        mkdir -p $DESTDIR/lib/modules/4.8.6-porteus/kernel/drivers/media/usb/uvc/
        cp uvcvideo.ko $DESTDIR/lib/modules/4.8.6-porteus/kernel/drivers/media/usb/uvc/
        mkdir -p $DESTDIR/etc/udev/rules.d/
        cp ../config/99-realsense-libusb.rules $DESTDIR/etc/udev/rules.d/
        cd ../.. ;;
    [Nn]* ) echo "Skipping...";;
    * ) echo "Please answer yes or no.";;
esac

echo "Module grabber libs"
read -p "Create and activate? yes/no:  " yn
case $yn in
    [Yy]* ) echo "building..."
        dir2xzm $DESTDIR /mnt/sda1/porteus/modules/grabbersLibs.1.0.xzm
        /opt/porteus-scripts/activate /mnt/sda1/porteus/modules/grabbersLibs.1.0.xzm;;
    [Nn]* ) echo "Skipping...";;
    * ) echo "Please answer yes or no.";;
esac


export DESTDIR=/opt/playground2
export PREFIX=/usr
export LIBDIR=/usr/lib64
export PATH=$DESTDIR/$PREFIX/usr/bin:$PATH
export INCLUDE_PATH=$DESTDIR/usr/include:$INCLUDE_PATH
export LIBRARY_PATH=$DESTDIR/$LIBDIR:$LIBRARY_PATH
export LD_LIBRARY_PATH=$DESTDIR/$LIBDIR:$LD_LIBRARY_PATH

echo "FLANN"
read -p "Build? yes/no:  " yn
case $yn in
    [Yy]* ) echo "building..."
        cp -r sources/flann-1.8.4-src.zip ./
        #FLANN
        unzip flann-1.8.4-src.zip 
        cd flann-1.8.4-src
        mkdir build
        cd build
        cmake .. -DCMAKE_INSTALL_PREFIX=$PREFIX \
            -DCMAKE_PREFIX_PATH=$DESTDIR$PREFIX \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_MATLAB_BINDINGS=OFF \
            -DBUILD_PYTHON_BINDINGS=OFF
        make -j5
        make install DESTDIR=$DESTDIR
        cd ../..
        mkdir -p $DESTDIR/$LIBDIR
        cp -r $DESTDIR/usr/lib/* $DESTDIR/$LIBDIR
        rm -rf $DESTDIR/usr/lib
        sed -i "s|/lib|/lib64|g" $DESTDIR/$LIBDIR/pkgconfig/flann.pc;;
    [Nn]* ) echo "Skipping...";;
    * ) echo "Please answer yes or no.";;
esac

#GDAL
# --with-python
echo "GDAL" #dep: not tested
read -p "Build? yes/no:  " yn
case $yn in
    [Yy]* ) echo "building..."
        cp -r sources/gdal-2.1.1.tar.xz ./
        tar xf gdal-2.1.1.tar.xz
        cd gdal-2.1.1
        ./configure --prefix=$PREFIX --libdir=$LIBDIR --with-netcdf --with-libtiff --with-sqlite3 --with-geotiff \
                    --with-mysql --with-curl --with-hdf5 --with-perl --with-geos \
                    --with-png --with-poppler --with-spatialite --with-openjpeg --enable-static=no
        make  -j5
        make install DESTDIR=$DESTDIR
        cd ..;;
    [Nn]* ) echo "Skipping...";;
    * ) echo "Please answer yes or no.";;
esac

#NetCDF-cxx
echo "NetCDF-cxx"
read -p "Build? yes/no:  " yn
case $yn in
    [Yy]* ) echo "building..."
        cp -r sources/netcdf-cxx-4.2.tar.gz ./
        export CPPFLAGS=-I$DESTDIR/usr/include 
        export LDFLAGS=-L$DESTDIR/$LIBDIR
        tar xf netcdf-cxx-4.2.tar.gz
        cd netcdf-cxx-4.2
        ./configure --prefix=$PREFIX --libdir=$LIBDIR --enable-shared
        make -j5
        make install DESTDIR=$DESTDIR
        export CPPFLAGS=
        export LDFLAGS=
        cd ..;;
    [Nn]* ) echo "Skipping...";;
    * ) echo "Please answer yes or no.";;
esac

echo "Module Prereq0"
read -p "Create and activate? yes/no:  " yn
case $yn in
    [Yy]* ) echo "building..."
        dir2xzm $DESTDIR /opt/prereq0.xzm
        /opt/porteus-scripts/activate /opt/prereq0.xzm;;
    [Nn]* ) echo "Skipping...";;
    * ) echo "Please answer yes or no.";;
esac

#VTK
echo "VTK" #dep: tcl tk gdal (for module Module_vtkIOGDAL), netcdf-cxx, libxml2, probably: gl2ps, hdf5, gcc-gfortran, flann but not tested
read -p "Build? yes/no:  " yn
case $yn in
    [Yy]* ) echo "building..."
        cp -r sources/VTK-7.0.0.tar.gz ./
        tar xf VTK-7.0.0.tar.gz
        cd VTK-7.0.0
        wget https://raw.githubusercontent.com/TomasBedrnik/ComputerVision/master/Porteus/patchCMakeLists.txt --no-check-certificate
        wget https://raw.githubusercontent.com/TomasBedrnik/ComputerVision/master/Porteus/patchGDAL.txt --no-check-certificate
        patch IO/GDAL/vtkGDALVectorReader.cxx < patchGDAL.txt
        patch CMakeLists.txt < patchCMakeLists.txt
        mkdir build
        cd build
        cmake .. \
            -Wno-dev \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_INSTALL_PREFIX=$PREFIX \
            -DVTK_QT_VERSION:STRING="5" \
            -DCMAKE_PREFIX_PATH=/usr/lib64/qt5/lib/cmake/ \
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
            -DVTK_USE_SYSTEM_LIBRARIES=OFF \
            -DVTK_USE_SYSTEM_GL2PS=OFF \
            -DVTK_USE_SYSTEM_HDF5=ON \
            -DModule_vtkGUISupportQtOpenGL:BOOL=ON \
            -DModule_vtkGUISupportQt:BOOL=ON \
            -DVTK_CUSTOM_LIBRARY_SUFFIX="" \
            -DVTK_INSTALL_INCLUDE_DIR:PATH=include/vtk \
            -DVTK_Group_Qt:BOOL=ON
        make -j5
        make install DESTDIR=$DESTDIR
        cd ../..  

        cp -r $DESTDIR/usr/lib/* $DESTDIR/$LIBDIR
        rm -rf $DESTDIR/usr/lib;;
    [Nn]* ) echo "Skipping...";;
    * ) echo "Please answer yes or no.";;
esac

#PCL
echo "PCL" #dep: eigen3, flann > 1.7.0, boost, VTK for visualization.
read -p "Build? yes/no:  " yn
case $yn in
    [Yy]* ) echo "building..."
        cp -r sources/pcl-1.8.0.tar.gz ./
        tar xf pcl-1.8.0.tar.gz
        cd pcl-pcl-1.8.0
        mkdir build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_PREFIX_PATH=$DESTDIR$PREFIX \
            -DCMAKE_INSTALL_PREFIX=$PREFIX \
            -DLIB_INSTALL_DIR=$LIBDIR
        make -j5
        make install DESTDIR=$DESTDIR
        cd ../..

        sed -i "s|//usr||g" $DESTDIR/$LIBDIR/pkgconfig/pcl_*.pc
        sed -i "s|//usr||g" $DESTDIR/usr/share/pcl-1.8/PCLConfig.cmake;;
    [Nn]* ) echo "Skipping...";;
    * ) echo "Please answer yes or no.";;
esac


echo "Module VTK PCL"
read -p "Create and activate? yes/no:  " yn
case $yn in
    [Yy]* ) echo "building..."
        dir2xzm $DESTDIR /mnt/sda1/porteus/modules/VTKPCL.1.0.xzm
        /opt/porteus-scripts/activate /mnt/sda1/porteus/modules/VTKPCL.1.0.xzm;;
    [Nn]* ) echo "Skipping...";;
    * ) echo "Please answer yes or no.";;
esac

export DESTDIR=/opt/playgroundOpenCV
export PREFIX=/usr
export LIBDIR=/usr/lib64
export PATH=$DESTDIR/$PREFIX/usr/bin:$PATH
export INCLUDE_PATH=$DESTDIR/usr/include:$INCLUDE_PATH
export LIBRARY_PATH=$DESTDIR/$LIBDIR:$LIBRARY_PATH
export LD_LIBRARY_PATH=$DESTDIR/$LIBDIR:$LD_LIBRARY_PATH

echo "OpenCV"
read -p "Build? yes/no:  " yn
case $yn in
    [Yy]* ) echo "building..."
        cp -r sources/3.1.0.zip ./
        cp -r sources/3.1.0.tar.gz ./
        
        unzip 3.1.0.zip
        tar xf 3.1.0.tar.gz
        cd opencv-3.1.0
        mkdir build
        cd build
        cmake .. \
            -DWITH_OPENNI2=ON \
            -DWITH_QT=ON \
            -DWITH_GDAL=ON \
            -DWITH_OPENGL=ON \
            -DWITH_TBB=ON \
            -DWITH_XINE=ON \
            -DWITH_GSTREAMER=OFF \
            -DBUILD_WITH_DEBUG_INFO=OFF \
            -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.1.0/modules \
            -DWITH_VTK=OFF \
            -DBUILD_TESTS=OFF \
            -DBUILD_PERF_TESTS=OFF \
            -DBUILD_EXAMPLES=OFF \
            -DINSTALL_C_EXAMPLES=OFF \
            -DINSTALL_PYTHON_EXAMPLES=OFF \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_INSTALL_PREFIX=$PREFIX \
            -DLIB_INSTALL_DIR=$LIBDIR \
            -DCMAKE_PREFIX_PATH=$DESTDIR$PREFIX \
            -DLIB_SUFFIX=64
        make -j5
        make install DESTDIR=$DESTDIR;;
    [Nn]* ) echo "Skipping...";;
    * ) echo "Please answer yes or no.";;
esac


echo "Module OpenCV"
read -p "Create and activate? yes/no:  " yn
case $yn in
    [Yy]* ) echo "building..."
        dir2xzm $DESTDIR /mnt/sda1/porteus/modules/OpenCV.3.1.0-1.xzm
        /opt/porteus-scripts/activate /mnt/sda1/porteus/modules/OpenCV.3.1.0-1.xzm;;
    [Nn]* ) echo "Skipping...";;
    * ) echo "Please answer yes or no.";;
esac
