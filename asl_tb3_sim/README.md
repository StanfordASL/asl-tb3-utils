# asl\_tb3\_sim
Simulation package for ASL Turtlebot3

## Launch File & Launch Argments
- `root.launch.py` -- launches simulation with SLAM algorithm
    - `x_init` -- Initial x position (default: -2.0)
    - `y_init` -- Initial y position (default: -0.5)
    - `world` -- Absolute path to an SDF world file (default: [turtlebot3\_world.sdf](worlds/turtlebot3_world.sdf))
- `<world>.launch.py` -- launches custom [world files](worlds)
    Note: `<world>` is chosen from [`arena`, `signs`, `maze`, `project_city`]
- `rviz.launch.py`
    - `config` -- Absolute path to an RVIZ configuration file (default: [default.rviz](configs/default.rviz))
