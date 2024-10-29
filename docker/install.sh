#!/bin/bash

#  Copyright (C) 2024 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

echo "Sourcing base image for full build..."
source /opt/ros/humble/setup.bash

mkdir -p ~/msgs && cd ~/msgs
git clone --depth 1 --single-branch -b ${GIT_BRANCH} https://github.com/usdot-fhwa-stol/carma-msgs.git

# Copy the necessary part for the carma-messenger
mkdir -p ~/carma-msgs && cd ~/carma-msgs
cp -r ~/msgs/carma-msgs/carma_cmake_common/ .
cp -r ~/msgs/carma-msgs/carma_msgs/ .

# Remove the carma_msgs since not necessary to build all of them
cd ~/ && sudo rm -r msgs

echo "Building carma_cmake_common"
colcon build --packages-select carma_cmake_common

echo "Building carma_msgs"
colcon build --packages-select carma_msgs

source install/setup.bash

echo "Building msger_mosaic_bridge"
colcon build --packages-select msger_mosaic_bridge --cmake-clean-cache 

echo "Build ROS2 msger_mosaic_bridge successfully"