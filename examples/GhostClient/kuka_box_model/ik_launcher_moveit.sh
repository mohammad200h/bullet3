#! /bin/bash

cd /home/$1/hand_RL_ws
source ./devel/setup.bash
cd /home/$1/hand_RL_ws/src/iiwa_pybullet_integration/pybullet_ik/launch

roslaunch hand_iiwa_detached.launch