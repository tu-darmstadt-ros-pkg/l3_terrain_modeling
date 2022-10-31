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
cd ${script_dir}
wget https://github.com/astumpf/l3_terrain_examples/archive/refs/heads/master.zip
echo

# Unpack files
echo_info ">>> Unpack files..."
unzip master.zip
echo

echo_info ">>> Move files..."
mkdir -p ../pcd/
mv -v l3_terrain_examples-master/pcd/* ../pcd/

mkdir -p ../bags/
mv -v l3_terrain_examples-master/bags/* ../bags/
echo

# Cleanup
echo_info ">>> Cleanup..."
rm -rf l3_terrain_examples-master
rm -f master.zip
echo
