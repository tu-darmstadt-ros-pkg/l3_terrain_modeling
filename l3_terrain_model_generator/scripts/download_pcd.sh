#!/bin/bash

set -e

script_dir="$(dirname $(readlink -f $0))"

source ${script_dir}/helper.sh

if [ -z "${ROS_DISTRO}" ]; then
    echo_error "No ROS workspace is sourced!"
    exit 1
fi

# Downloading pcd files
echo_info ">>> Downloading pcd files..."
mkdir -p ${script_dir}/../pcd
cd ${script_dir}/../pcd
wget http://www.sim.informatik.tu-darmstadt.de/~stumpf/footstep_planning_test_pcl.zip
echo

# Unpack files
echo_info ">>> Unpack files..."
unzip footstep_planning_test_pcl.zip
mv -v pcl/* .
echo

# Cleanup
echo_info ">>> Cleanup..."
rm -rf pcl
rm -f footstep_planning_test_pcl.zip
echo
