#!/bin/sh
SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`
export LD_LIBRARY_PATH=$SCRIPTPATH/rtabmap
$SCRIPTPATH/rtabmap/rtabmap-rgbd_mapping "$@"
