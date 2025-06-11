#!/bin/bash
# This script removes the existing gz submodule and adds the forked version.

# Exit immediately if a command exits with a non-zero status.
set -e

echo "Deinitializing the submodule..."
git submodule deinit Tools/simulation/gz

echo "Removing the submodule..."
git rm Tools/simulation/gz

echo "Committing the removal..."
git commit -m "Removed gz submodule"

echo "Deleting cached submodule data..."
rm -rf .git/modules/Tools/simulation/gz

echo "Adding the new gz submodule..."
git submodule add -b main https://github.com/ingeborgaarnes/PX4-gazebo-models.git Tools/simulation/gz

echo "Committing the new submodule addition..."
git commit -m "Added forked gz submodule"

echo "Initializing submodules..."
git submodule init

echo "Updating submodules..."
git submodule update 

echo "Displaying submodule status..."
git submodule status

echo "Update again..."
git submodule update --init --recursive

echo "Run program..."
make px4_sitl

