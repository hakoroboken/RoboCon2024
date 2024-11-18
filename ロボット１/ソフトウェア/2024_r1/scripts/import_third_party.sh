#!/bin/bash

sudo apt-get install python3-vcstool -y

echo "Install ThirdParty Repos"

vcs import ./src/2024_r1/thirdparty < ./src/2024_r1/thirdparty/thirdparty.repos