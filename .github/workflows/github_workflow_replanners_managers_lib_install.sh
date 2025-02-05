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
cd $WORKSPACE_DIR

# Clone deps into the src folder 
cd $SRC_DIRc

echo "Cloning repositories into $SRC_DIR"
git clone https://github.com/JRL-CARI-CNR-UNIBS/graph_core.git
git clone https://github.com/JRL-CARI-CNR-UNIBS/graph_display.git
git clone https://github.com/JRL-CARI-CNR-UNIBS/moveit_collision_checker.git 

git clone https://github.com/JRL-CARI-CNR-UNIBS/cnr_scene_manager.git 
git clone https://github.com/CNR-STIIMA-IRAS/cnr_tf_named_object_loader.git 

git clone https://github.com/CNR-STIIMA-IRAS/subscription_notifier.git 

git clone https://github.com/JRL-CARI-CNR-UNIBS/replanners_lib.git 
git clone https://github.com/JRL-CARI-CNR-UNIBS/trajectories_processors_lib.git 
git clone https://github.com/JRL-CARI-CNR-UNIBS/replanners_managers_lib.git 

# Build the workspace
cd $WORKSPACE_DIR
echo "Building the Catkin workspace"
catkin init
catkin config --extend /opt/ros/noetic
catkin config --install

rosdep update
rosdep install --from-paths src --ignore-src -r -y

catkin build --verbose -cs

# Source the workspace setup
echo "Sourcing workspace setup"
source $WORKSPACE_DIR/devel/setup.bash
