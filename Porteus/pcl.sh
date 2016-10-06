#!/usr/bin/sh
#############
#SUDO needs to be configured to work withou password
#It isnt much safe so you can disable it after compilation
############
#Needs subversion (with dependencies) and git modules
############

export PREFIX=/opt/playground2
export PATH=$PREFIX/bin:$PATH
export INCLUDE_PATH=$PREFIX/include:$INCLUDE_PATH
export LIBRARY_PATH=$PREFIX/lib:$LIBRARY_PATH
export LD_LIBRARY_PATH=$PREFIX/lib:$LD_LIBRARY_PATH


