import os
import shutil
import sys

source_file = "simulator.launch"
target_path = "/home/clover/catkin_ws/src/clover/clover_simulation/launch/simulator.launch"

if not os.path.exists(source_file):
    print(f"Error: {source_file} not found")
    sys.exit(1)

try:
    os.makedirs(os.path.dirname(target_path), exist_ok=True)
    
    if os.path.exists(target_path):
        shutil.copy2(target_path, f"{target_path}.backup")
    
    shutil.copy2(source_file, target_path)
    print("Launch updated successfully")
    sys.exit(0)
    
except Exception as e:
    print(f"Error: {e}")
    sys.exit(1)