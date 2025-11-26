#!/bin/bash

echo "Generating map"
python3 gen.py

echo "Generating map"
sleep 5

echo "Run simulator"
/bin/bash -c 'source /home/clover/catkin_ws/devel/setup.bash; roslaunch clover_simulation simulator.launch'