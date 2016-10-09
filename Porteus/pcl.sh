#!/bin/sh
#############
#SUDO needs to be configured to work withou password
#It isnt much safe so you can disable it after compilation
############
#Needs subversion (with dependencies) and git modules
############
#Modules: openldap-client

export PREFIX=/opt/playgroundPCL
export PATH=$PREFIX/bin:$PREFIX/usr/bin:$PATH
export INCLUDE_PATH=$PREFIX/usr/include:$PREFIX/include:$INCLUDE_PATH
export LIBRARY_PATH=$PREFIX/lib:$PREFIX/usr/lib:$LIBRARY_PATH
export LD_LIBRARY_PATH=$PREFIX/lib:$PREFIX/usr/lib:$LD_LIBRARY_PATH


#FLANN
wget http://www.cs.ubc.ca/research/flann/uploads/FLANN/flann-1.8.4-src.zip --no-check-certificate
unzip flann-1.8.4-src.zip 
cd flann-1.8.4-src
mkdir build
cd build
export HDF5_hdf5_cpp_LIBRARY=$PREFIX/usr/include/
cmake .. -DCMAKE_INSTALL_PREFIX=$PREFIX/usr \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_MATLAB_BINDINGS=OFF \
      -DBUILD_PYTHON_BINDINGS=OFF
make
sudo make install
cd ../..

#TCL
wget http://downloads.sourceforge.net/sourceforge/tcl/tcl8.6.6-src.tar.gz --no-check-certificate
tar xf tcl8.6.6-src.tar.gz
cd tcl8.6.6/unix
./configure --prefix=$PREFIX/usr
make
sudo make install
cd ../..
#TK
wget http://downloads.sourceforge.net/sourceforge/tcl/tk8.6.6-src.tar.gz --no-check-certificate
tar xf tk8.6.6-src.tar.gz
cd tk8.6.6/unix
./configure --prefix=$PREFIX/usr
make
sudo make install
cd ../..

#HDF5
wget https://support.hdfgroup.org/ftp/HDF5/current/src/hdf5-1.8.17.tar.bz2 --no-check-certificate
tar xf hdf5-1.8.17.tar.bz2
cd hdf5-1.8.17
./configure --prefix=$PREFIX/usr
make
sudo make install
cd ..

#NetCDF
export CPPFLAGS=-I$PREFIX/usr/include 
export LDFLAGS=-L$PREFIX/usr/lib 
wget ftp://ftp.unidata.ucar.edu/pub/netcdf/netcdf-4.4.1.tar.gz
tar xf netcdf-4.4.1.tar.gz
cd netcdf-4.4.1
./configure --prefix=$PREFIX/usr --enable-shared --enable-netcdf-4 --enable-dap-netcdf
make
sudo make install
cd ..


#NetCDF-cxx
export CPPFLAGS=-I$PREFIX/usr/include 
export LDFLAGS=-L$PREFIX/usr/lib 
wget ftp://ftp.unidata.ucar.edu/pub/netcdf/netcdf-cxx-4.2.tar.gz
tar xf netcdf-cxx-4.2.tar.gz
cd netcdf-cxx-4.2
./configure --prefix=$PREFIX/usr
make
sudo make install
cd ..

#GL2PS
wget http://geuz.org/gl2ps/src/gl2ps-1.3.9.tgz --no-check-certificate
tar xf gl2ps-1.3.9.tgz
cd gl2ps-1.3.9-source
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$PREFIX/usr
make
sudo make install
cd ../..

#GDAL
wget http://download.osgeo.org/gdal/2.1.1/gdal-2.1.1.tar.xz --no-check-certificate
tar xf gdal-2.1.1.tar.xz
cd gdal-2.1.1
./configure --prefix=$PREFIX/usr --with-netcdf --with-libtiff --with-sqlite3 --with-geotiff \
              --with-mysql --with-python --with-curl --with-hdf5 --with-perl --with-geos \
              --with-png --with-poppler --with-spatialite --with-openjpeg
make 
sudo make install
cd ..

#libjson
#git clone git://github.com/vincenthz/libjson/
#cd libjson
#make CFLAGS:='-Wall -Os -fPIC'
#sudo make install PREFIX:=$PREFIX/usr

#jsoncpp
git clone https://github.com/open-source-parsers/jsoncpp
cd jsoncpp
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$PREFIX/usr \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_STATIC_LIBS=OFF
make
sudo make install
cd ../..

#VTK
wget http://www.vtk.org/files/release/7.0/VTK-7.0.0.tar.gz --no-check-certificate
tar xf VTK-7.0.0.tar.gz
cd VTK-7.0.0
#git clone https://github.com/Kitware/VTK.git
#cd VTK
mkdir build
cd build
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
      -DVTK_WRAP_PYTHON=OFF \
      -DVTK_WRAP_TCL=ON \
      -DVTK_USE_SYSTEM_LIBRARIES=ON \
      -DVTK_USE_SYSTEM_GL2PS=OFF \
      -DVTK_USE_SYSTEM_HDF5=ON
make
sudo make install
cd ../..   
      
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz --no-check-certificate
tar xf pcl-1.8.0.tar.gz
cd pcl-pcl-1.8.0
#git clone https://github.com/PointCloudLibrary/pcl pcl-trunk
#cd pcl-trunk


#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#VYRESIT OPENNI !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#Problem s flann -> najde ho to v /usr/include, ale tam nema vubec bejt§!!
# NAKORPířovat openni + pkgconfgi
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

mkdir build && cd build
#export OPENNI2_INCLUDE=/home/guest/Desktop/openni2/OpenNI-Linux-x64-2.2/Include
#export OPENNI2_REDIST=/home/guest/Desktop/openni2/OpenNI-Linux-x64-2.2/Redist

cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX/usr
#\
#      -DFLANN_INCLUDE_DIR=$PREFIX/usr/include/flann \
#      -DFLANN_LIBRARY=$PREFIX/usr/lib\
#      -DOPENNI2_LIBRARY=/home/guest/Desktop/openni2/OpenNI-Linux-x64-2.2/Redist \
#      -DOPENNI2_INCLUDE_DIRS=/home/guest/Desktop/openni2/OpenNI-Linux-x64-2.2/Include
      
      #\
      #-DFLANN_INCLUDE_DIR=$PREFIX/usr/include/flann \
      #-DFLANN_LIBRARY=$PREFIX/usr/lib
      #\
      #-DVTK_DIR=/home/guest/Desktop/pclNew/VTK-7.0.0/build
make
sudo make install
cd ../..
sudo dir2xzm $PREFIX /mnt/sda1/porteus/modules/pcl.xzm