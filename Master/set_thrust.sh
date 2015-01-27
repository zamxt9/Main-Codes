#!/bin/bash

if [ -z "$1" ]; then
	echo "usage: set_thrust.sh <thrust>"
    exit
fi

Thrust=$1

rostopic pub DesiredThrust std_msgs/Float32 $Thrust
