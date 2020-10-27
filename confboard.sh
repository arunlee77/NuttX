#!/bin/bash
echo make distclean
make distclean
echo ./tools/configure.sh ./configs/deliboz/nsh
./tools/configure.sh ./configs/deliboz/nsh

# local setup
echo cd /root/uros_ws
cd /root/uros_ws
echo . ./install/local_setup.sh
. ./install/local_setup.sh 
#echo cd uros_ws/firmware/NuttX
#cd uros_ws/firmware/NuttX
