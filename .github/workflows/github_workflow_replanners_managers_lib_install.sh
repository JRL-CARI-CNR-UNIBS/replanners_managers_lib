#!/bin/bash
set -e

# Source ROS Noetic setup
echo "Sourcing ROS Noetic"
source /opt/ros/noetic/setup.bash

# Define workspace paths
WORKSPACE_DIR=$(pwd)/openmore_ws
SRC_DIR=$WORKSPACE_DIR/src

# Create the workspace and source folder
echo "Setting up Catkin workspace at $WORKSPACE_DIR"
mkdir -p $SRC_DIR

# Clone deps into the src folder 
cd $SRC_DIR

echo "Cloning repositories into $SRC_DIR"
git clone https://github.com/JRL-CARI-CNR-UNIBS/replanners_managers_lib.git 

vcs import < replanners_managers_lib/deps.repos

# Build the workspace
cd $WORKSPACE_DIR
echo "Building the Catkin workspace"
catkin init
catkin config --extend /opt/ros/noetic
catkin config --install
catkin config --cmake-args -DUSE_ROS1=OFF -DBUILD_UNIT_TESTS=OFF

rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "Building.."
catkin build -cs
echo "Successfully built the ws!"

# Source the workspace setup
echo "Sourcing workspace setup"
source $WORKSPACE_DIR/devel/setup.bash
