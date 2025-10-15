from pathlib import Path
import numpy as np
import os
import pybullet as p

def get_default_warehouse_params():
    # Layout partially adapted from here
    # https://ojs.aaai.org/index.php/SOCS/article/view/31593/33753

    rows, cols = 30, 36
    # rows, cols = 9, 14

    # Setup workstations
    work_stns = np.zeros([rows, cols])
    work_stns[1:-1:3,
              -1] = 1
    work_stns[1:-1:3,
              0] = 1

    # Setup shelves
    # Shelves are purely DECORATIVE. See endpoints for the actual robot destination.
    shelves = np.zeros([rows, cols])
    shelves[2:-2:4,
            1: -2] = 1
    shelves[3:-2:4,
            1: -2] = 1
    
    shelves[2:-2:4,
            1:-2:11] = 0
    shelves[3:-2:4,
            1:-2:11] = 0
    
    # Setup endpoints. The destination to collect a delivery is one of these endpoints.
    endpoints = np.zeros([rows, cols])
    endpoints[1:-1:4,
              1: -2] = 1
    endpoints[4:-1:4,
              1: -2] = 1

    endpoints[1:-1:4,
              1:-2:11] = 0
    endpoints[4:-1:4,
              1:-2:11] = 0
    
    return rows, cols, work_stns, shelves, endpoints



# Code adapted from the default course project
def create_struct_urdf(
    struct_map,
    urdf_path: str,
    grid_xy: float = 1.0,
    grid_z: float = 3.0,
    struct_center_x: float = 0.0,
    struct_center_y: float = 0.0,
    struct_center_z: float = 0.0,
    box_color=(1, 1, 1, 1),
    has_collison=True,
):
    """
    Generate a URDF file for a struct (walls as cubes) that can be loaded in PyBullet.
    - Single root link 'struct_base'
    - Each block attached with a fixed joint
    """

    n_rows = len(struct_map)
    n_cols = len(struct_map[0]) if n_rows > 0 else 0
    half_x, half_y, half_z = grid_xy / 2, grid_xy / 2, grid_z / 2

    urdf_parts = [
        '<?xml version="1.0" ?>',
        '<robot name="struct">',
        '  <link name="struct_base"/>',   # single root link
    ]

    cube_positions = []

    for i in range(n_rows):
        for j in range(n_cols):
            if struct_map[i][j] == 1:
                world_x = struct_center_x + (j - n_cols / 2 + 0.5) * grid_xy
                world_y = struct_center_y + (n_rows / 2 - i - 0.5) * grid_xy
                world_z = struct_center_z + half_z

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
    <inertial>
      <mass value="0"/> <!-- static -->
      <inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/>
    </inertial>
"""
                if has_collison:
                    link += f"""
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="{grid_xy} {grid_xy} {grid_z}"/>
      </geometry>
    </collision>
"""

                joint = f"""
  <joint name="joint_{name}" type="fixed">
    <parent link="struct_base"/>
    <child link="{name}"/>
    <origin xyz="{world_x} {world_y} {world_z}" rpy="0 0 0"/>
  </joint>
"""
                link += "</link>"
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