#!/usr/bin/env sh

echo "Update system packages"
sudo apt -qq update && sudo apt upgrade -y

echo "Pulling updates for asl-tb3-utils"
cd ~/tb_ws/src/asl-tb3-utils
git checkout main && git pull

echo "Pulling updates for asl-tb3-driver"
cd ~/tb_ws/src/asl-tb3-driver
git checkout main && git pull

echo "Re-building workspace"
cd ~/tb_ws
GZ_VERSION=garden colcon build --symlink-install
