from pathlib import Path
import numpy as np
import os
import pybullet as p

def init_scene(rows, cols, work_stn_arr, shelves_arr):
    planeId = p.loadURDF("assets/plane/plane.urdf")

    whouse_map = np.zeros([rows + 2, cols + 2])

    # Create border walls
    whouse_map[0,:] = 1
    whouse_map[-1,:] = 1
    whouse_map[:,0] = 1
    whouse_map[:,-1] = 1

    cube_pos = create_whouse_urdf(whouse_map, "wh.urdf")
    work_stns_pos = create_whouse_urdf(work_stn_arr, "endpoints.urdf", grid_z=1.25, box_color=(1, 0, 0.5, 0.3))
    shelves_pos = create_whouse_urdf(shelves_arr, "shelves.urdf", grid_z=1, box_color=(0, 0, 0.5, 0.3))
    print(cube_pos)

    wh = p.loadURDF("wh.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
    endpoints = p.loadURDF("endpoints.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
    shelves = p.loadURDF("shelves.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)


def create_whouse_urdf(
    whouse_map,
    urdf_path: str,
    grid_xy: float = 1.0,
    grid_z: float = 3.0,
    whouse_center_x: float = 0.0,
    whouse_center_y: float = 0.0,
    whouse_center_z: float = 0.0,
    box_color=(1, 1, 1, 1),
):
    """
    Generate a URDF file for a whouse (walls as cubes) that can be loaded in PyBullet.
    - Single root link 'whouse_base'
    - Each block attached with a fixed joint
    """

    n_rows = len(whouse_map)
    n_cols = len(whouse_map[0]) if n_rows > 0 else 0
    half_x, half_y, half_z = grid_xy / 2, grid_xy / 2, grid_z / 2

    urdf_parts = [
        '<?xml version="1.0" ?>',
        '<robot name="whouse">',
        '  <link name="whouse_base"/>',   # single root link
    ]

    cube_positions = []

    for i in range(n_rows):
        for j in range(n_cols):
            if whouse_map[i][j] == 1:
                world_x = whouse_center_x + (j - n_cols / 2 + 0.5) * grid_xy
                world_y = whouse_center_y + (n_rows / 2 - i - 0.5) * grid_xy
                world_z = whouse_center_z + half_z

                name = f"block_{i}_{j}"

                # child link
                link = f"""
  <link name="{name}">
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="{grid_xy} {grid_xy} {grid_z}"/>
      </geometry>
      <material name="wall">
        <color rgba="{box_color[0]} {box_color[1]} {box_color[2]} {box_color[3]}"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="{grid_xy} {grid_xy} {grid_z}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0"/> <!-- static -->
      <inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
"""

                joint = f"""
  <joint name="joint_{name}" type="fixed">
    <parent link="whouse_base"/>
    <child link="{name}"/>
    <origin xyz="{world_x} {world_y} {world_z}" rpy="0 0 0"/>
  </joint>
"""

                urdf_parts.append(link)
                urdf_parts.append(joint)
                cube_positions.append((world_x, world_y, world_z))

    urdf_parts.append("</robot>")
    urdf_str = "\n".join(urdf_parts)

    Path(os.path.dirname(urdf_path) or ".").mkdir(parents=True, exist_ok=True)
    with open(urdf_path, "w") as f:
        f.write(urdf_str)

    cube_positions = np.array(cube_positions) if len(cube_positions) else None
    return cube_positions