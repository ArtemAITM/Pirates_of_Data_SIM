import math
import random
import os


def generate_urdf_with_branches():
    total_length = random.uniform(5.0, 10.0)
    angle_degrees = random.uniform(-30.0, 30.0)
    angle_rad = math.radians(angle_degrees)

    first_length = total_length * random.uniform(0.3, 0.8)
    second_length = total_length - first_length
    overlap_length = 0.14
    base_x, base_y = 1.0, 1.0
    branches = []
    target_branches = 5
    start_margin = 0.5
    end_margin = 0.5
    available_length = total_length - start_margin - end_margin
    
    if available_length < (target_branches - 1) * 1.0:
        target_branches = int(available_length / 1.0) + 1
    
    if target_branches > 1:
        spacing = available_length / (target_branches - 1)
    else:
        spacing = 0
    
    for i in range(target_branches):
        global_pos = start_margin + i * spacing
        
        if global_pos < first_length:
            segment = "first"
            local_pos = global_pos
        else:
            segment = "second"
            local_pos = global_pos - first_length + overlap_length
        
        branch_length = random.uniform(0.75, 2.0)
        branch_direction = random.choice([-1, 1])
        
        branches.append({
            "segment": segment,
            "global_pos": global_pos,
            "position": local_pos,
            "length": branch_length,
            "direction": branch_direction
        })

    global_rotation_rad = math.radians(45.0)

    urdf_template = f'''<?xml version="1.0"?>
<robot name="generated_robot_with_branches">
  <link name="base_footprint"/>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="{base_x} {base_y} 0.1" rpy="0 0 {global_rotation_rad}"/>
  </joint>
  <link name="base_link">
    <visual>
      <origin xyz="{first_length/2} 0 0" rpy="0 1.570796 0"/>
      <geometry>
        <cylinder radius="0.1" length="{first_length}"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="{first_length/2} 0 0" rpy="0 1.570796 0"/>
      <geometry>
        <cylinder radius="0.1" length="{first_length}"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="{first_length/2} 0 0" rpy="0 1.570796 0"/>
      <mass value="1"/>
      <inertia ixx="0.16666666666666666" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.16666666666666666"/>
    </inertial>
  </link>
  <gazebo reference="base_link">
    <material>Gazebo/Red</material>
  </gazebo>
  
  <joint name="base_link_to_cylinder1" type="fixed">
    <parent link="base_link"/>
    <child link="cylinder1"/>
    <origin xyz="{first_length - overlap_length} 0 0" rpy="0 0 {angle_rad}"/>
  </joint>
  <link name="cylinder1">
    <visual>
      <origin xyz="{second_length/2} 0 0" rpy="0 1.570796 0"/>
      <geometry>
        <cylinder radius="0.1" length="{second_length}"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="{second_length/2} 0 0" rpy="0 1.570796 0"/>
      <geometry>
        <cylinder radius="0.1" length="{second_length}"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="{second_length/2} 0 0" rpy="0 1.570796 0"/>
      <mass value="1"/>
      <inertia ixx="0.75" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.75"/>
    </inertial>
  </link>
  <gazebo reference="cylinder1">
    <material>Gazebo/Red</material>
  </gazebo>'''

    for i, branch in enumerate(branches):
        branch_name = f"branch_{i+1}"

        if branch["segment"] == "first":
            joint_xyz = f'{branch["position"]} 0 0'
            joint_rpy = f'0 0 {1.570796 * branch["direction"]}'
            parent_link = "base_link"
        else:
            joint_xyz = f'{branch["position"]} 0 0'
            joint_rpy = f'0 0 {1.570796 * branch["direction"]}'
            parent_link = "cylinder1"

        urdf_template += f'''
  <joint name="{parent_link}_to_{branch_name}" type="fixed">
    <parent link="{parent_link}"/>
    <child link="{branch_name}"/>
    <origin xyz="{joint_xyz}" rpy="{joint_rpy}"/>
  </joint>
  <link name="{branch_name}">
    <visual>
      <origin xyz="{branch["length"]/2} 0 0" rpy="0 1.570796 0"/>
      <geometry>
        <cylinder radius="0.05" length="{branch["length"]}"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="{branch["length"]/2} 0 0" rpy="0 1.570796 0"/>
      <geometry>
        <cylinder radius="0.05" length="{branch["length"]}"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="{branch["length"]/2} 0 0" rpy="0 1.570796 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <gazebo reference="{branch_name}">
    <material>Gazebo/Red</material>
  </gazebo>'''

    urdf_template += '''
</robot>'''
    
    return urdf_template


try:
    urdf_content = generate_urdf_with_branches()
    home_dir = os.path.expanduser('~')
    urdf_path = os.path.join(
        home_dir, 'catkin_ws/src/clover/clover/data/urdf/generated.urdf')
    os.makedirs(os.path.dirname(urdf_path), exist_ok=True)
    with open(urdf_path, 'w') as f:
        f.write(urdf_content)

    print(f"\nURDF successfully saved to: {urdf_path}")
    
except Exception as e:
    print(f"Error: {e}")