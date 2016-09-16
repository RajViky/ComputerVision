#!/usr/bin/sh
#############
#SUDO needs to be configured to work withou password
#It isnt much safe so you can disable it after compilation
############
#Needs subversion (with dependencies) and git modules
############

export PREFIX=/opt/playground
#Place new things over the system
export PATH=$PREFIX/bin:$PATH
export INCLUDE_PATH=$PREFIX/include:$PREFIX/include/c++/6.2.1/:$INCLUDE_PATH
export LIBRARY_PATH=$PREFIX/lib:$LIBRARY_PATH
export LD_LIBRARY_PATH=$PREFIX/lib:$LD_LIBRARY_PATH
export C_INCLUDE_PATH=$PREFIX/include:$PREFIX/include/c++/6.2.1/:$C_INCLUDE_PATH
export CPLUS_INCLUDE_PATH=$PREFIX/include:$PREFIX/include/c++/6.2.1/:$CPLUS_INCLUDE_PATH
#GCC
svn checkout svn://gcc.gnu.org/svn/gcc/branches/gcc-6-branch/ gcc
mkdir gccobj
cd gccobj
../gcc/configure --enable-languages=c,c++ \
    --prefix=$PREFIX \
    --disable-multilib \
    --with-system-zlib \
    --with-default-libstdcxx-abi=gcc4-compatible
make bootstrap MAKE="make -j 4"
sudo make install
cd ..

#Python 2.7.12
wget https://www.python.org/ftp/python/2.7.12/Python-2.7.12.tgz --no-check-certificate
tar -xvf Python-2.7.12.tgz
cd Python-2.7.12
./configure --enable-shared --prefix=$PREFIX
make -j 4
sudo make install
cd ..
#Setuptools
wget https://pypi.python.org/packages/87/ba/54197971d107bc06f5f3fbdc0d728a7ae0b10cafca46acfddba65a0899d8/setuptools-27.2.0.tar.gz#md5=b39715612fdc0372dbfd7b3fcf5d4fe5 --no-check-certificate
tar -xzf setuptools-27.2.0.tar.gz                                                                                                                                                                                                                                                                                                     
cd setuptools-27.2.0 
sudo env LD_LIBRARY_PATH=$PREFIX/lib python setup.py install --prefix=$PREFIX
cd ..

#PIP
wget https://bootstrap.pypa.io/get-pip.py --no-check-certificate
sudo env LD_LIBRARY_PATH=$PREFIX/lib python get-pip.py --prefix $PREFIX

#Cython
sudo env LD_LIBRARY_PATH=$PREFIX/lib pip install Cython --prefix $PREFIX

#Numpy
git clone git://github.com/numpy/numpy.git numpy
cd numpy
python setup.py build
sudo env LD_LIBRARY_PATH=$PREFIX/lib python setup.py install --prefix=$PREFIX
cd ..

#Eigen
wget http://bitbucket.org/eigen/eigen/get/3.2.9.tar.bz2 --no-check-certificate
tar -xjf 3.2.9.tar.bz2 
cd eigen-eigen-dc6cfdf9bcec
mkdir build_dir
cd build_dir
cmake -DCMAKE_INSTALL_PREFIX=$PREFIX ..
sudo make install
cd ../..

#GLEW
git clone https://github.com/nigels-com/glew.git glew
cd glew/auto
make
cd ..
make
sudo cp -R ./bin/* $PREFIX/bin/
sudo cp -R ./include/* $PREFIX/include/
sudo cp -R ./lib/* $PREFIX/lib/
sudo cp -R ./lib/* $PREFIX/lib64/ 
cd ..

#BOOST
wget http://downloads.sourceforge.net/project/boost/boost/1.61.0/boost_1_61_0.tar.bz2 --no-check-certificate
tar -xjf boost_1_61_0.tar.bz2
cd boost_1_61_0
./bootstrap.sh --prefix=$PREFIX
./b2
sudo ./b2 install --prefix=$PREFIX
cd..

#LAPACK
#GET FROM USM -> with dependencies

#OpenCV 2.4.13
wget http://github.com/Itseez/opencv/archive/2.4.13.zip --no-check-certificate
unzip 2.4.13.zip
cd opencv-2.4.13
mkdir release
cd release
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$PREFIX cmake -DENABLE_PRECOMPILED_HEADERS=OFF ..
make
sudo make install
cd ../..

#Pangloin
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
make -j 4
