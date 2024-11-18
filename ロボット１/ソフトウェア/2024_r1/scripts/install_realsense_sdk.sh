#!/bin/bash

echo "Set Public key"

sudo mkdir -p /etc/apt/keyrings

curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update -y

sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg