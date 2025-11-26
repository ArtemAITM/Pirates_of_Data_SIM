#!/bin/bash

bash install_libs.txt
sleep 2
python3 gen.py
sleep 2
python3 markers.py
sleep 2
python3 update_launch.py
