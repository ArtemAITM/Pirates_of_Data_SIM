import os

def generate_marker_map():
    markers = []
    
    for marker_id in range(100):
        if marker_id == 17:
            continue

        x = marker_id % 10
        y = 9 - (marker_id // 10)
        
        length = 0.44
        z = 0.0
        rot_z = 0.0
        rot_y = 0.0
        rot_x = 0.0
        
        markers.append({
            'id': marker_id,
            'length': length,
            'x': x,
            'y': y, 
            'z': z,
            'rot_z': rot_z,
            'rot_y': rot_y,
            'rot_x': rot_x
        })
    
    return markers

def save_marker_map(markers, filename):
    with open(filename, 'w') as f:
        f.write("# id\tlength\tx\ty\tz\trot_z\trot_y\trot_x\n")

        for marker in markers:
            f.write(f"{marker['id']}\t{marker['length']:.2f}\t{marker['x']}\t{marker['y']}\t{marker['z']}\t{marker['rot_z']}\t{marker['rot_y']}\t{marker['rot_x']}\n")

def update_launch1_file(launch_file_path, marker_filename):
    with open(launch_file_path, 'r') as f:
        content = f.read()
    
    content = content.replace('default="map.txt"', f'default="$(find clover)/data/{marker_filename}"')
    content = content.replace('default="0.22"', 'default="0.44"')
    content = content.replace('<arg name="aruco_detect" default="false"/>', '<arg name="aruco_detect" default="true"/>')
    content = content.replace('<arg name="aruco_map" default="false"/>', '<arg name="aruco_map" default="true"/>')
    content = content.replace('arg name="aruco_vpe" default="false"/>', 'arg name="aruco_vpe" default="true"/>')
    content = content.replace('<param name="map" value="$(find aruco_pose)/map/$(arg map)"/>', '<param name="map" value="$(arg map)"/>')
    with open(launch_file_path, 'w') as f:
        f.write(content)


def update_launch2_file(launch_file_path, marker_filename):
    with open(launch_file_path, 'r') as f:
        content = f.read()
    
    content = content.replace('<arg name="aruco" default="false"/>', '<arg name="aruco" default="true"/>')
    with open(launch_file_path, 'w') as f:
        f.write(content)


markers = generate_marker_map()
home_dir = os.path.expanduser('~')
path = os.path.join(home_dir, 'catkin_ws/src/clover/clover/data/marker_map.txt')
os.makedirs(os.path.dirname(path), exist_ok=True)
save_marker_map(markers, path)

launch_file_path = os.path.join(os.path.join(home_dir, "catkin_ws/src/clover/clover/launch/"), "aruco.launch")
if os.path.exists(launch_file_path):
    update_launch1_file(launch_file_path, "marker_map.txt")
    print(f"Launch update: {launch_file_path}")
else:
    print(f"Launch not: {launch_file_path}")

launch_file_path = os.path.join(os.path.join(home_dir, "catkin_ws/src/clover/clover/launch/"), "clover.launch")
if os.path.exists(launch_file_path):
    update_launch2_file(launch_file_path, "marker_map.txt")
    print(f"Launch update: {launch_file_path}")
else:
    print(f"Launch not: {launch_file_path}")
