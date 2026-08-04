# Crowd Navigation in a Multi-Room Environment: a Model Predictive Control Framework for Mobile Robots
This repository provides a sensor-based Model Predictive Control (MPC) framework for safe mobile robot navigation among crowds in non-convex, multi-room environments. The free space is decomposed into a set of overlapping convex regions that form a topological graph, from which a high-level planner computes a sequence of traversable areas toward the goal. A perception pipeline fuses 2D LiDAR data with semantic information from an RGB-D camera and uses Kalman filters to estimate and predict the motion of the surrounding humans. These predictions feed a Nonlinear MPC (NMPC) controller that generates the robot commands, enforcing safety through discrete-time Control Barrier Functions (DT-CBFs) so that the robot avoids collisions while remaining within the navigable regions. The approach has been validated in high-fidelity Gazebo simulations and in real-world experiments on the TIAGo mobile robot.

For further details, including the paper and videos of simulations and experiments, visit the [project page](https://gianni0907.github.io/crowd_navigation/).

## Installation
This project runs on ROS Noetic (Ubuntu 20.04). It is recommended to run it inside a Docker container, set up by following the guide below. The resulting container already ships everything needed to build and run this code — ROS Noetic, the PAL/TIAGo simulation stack, **acados** (with its `blasfeo`/`hpipm` backends), and the required Python packages — so no manual dependency installation is needed.

**https://github.com/DIAG-Robotics-Lab/labrob_tiago_docker**

> The Python packages used by this code (`numpy<2`, `scipy`, `casadi`, `shapely`, `scikit-learn`, `opencv-python`, `ultralytics`) are listed in `requirements.txt` for reference — they are already present in the image.

### Build
The container is launched with `rocker --home`, which mounts your host home directory inside the container. Therefore, clone this repository on the **host** first, then build it from **inside** the container.

1. On the host, clone this repository into the `src` folder of a catkin workspace under your home directory:
   ```
   mkdir -p ~/crowdnav_ws/src
   cd ~/crowdnav_ws/src
   git clone https://github.com/gianni0907/crowdnav
   ```
2. Create and enter the container as described in the guide above. Because your home is mounted, `~/crowdnav_ws` is available inside it.
3. From inside the container, build in *Release* mode and source the workspace:
   ```
   cd ~/crowdnav_ws
   catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
   catkin build
   source devel/setup.bash
   ```

## Configuration
Before running, review the key parameters in `crowd_navigation_core/src/crowd_navigation_core/Hparams.py`:

- `simulation`: set to `True` for Gazebo, `False` for the real robot.
- `world_type`: defines the navigable area structure. For simulation, use one of `WorldType.EMPTY`, `WorldType.TWO_ROOMS`, `WorldType.THREE_ROOMS`, or `WorldType.CORRIDOR`, which correspond to the provided Gazebo worlds. `WorldType.DIAG` and `WorldType.HALL` were defined for specific real-world experiments and are included as examples; for a different real environment, add a new entry to the `WorldType` enum and define the corresponding convex regions in `Hparams.py`. This must match the Gazebo world being launched when running in simulation.
- `n_agents`: total number of actors in the scene (used for simulation logging/plotting). Update this to match the chosen world.
- `perception`: sensing mode, one of `Perception.GTRUTH` (ground truth, simulation only), `Perception.LASER`, `Perception.CAMERA`, or `Perception.BOTH`.

If the real robot is used (`simulation=False`), make sure it is inside the admissible region defined by `area0` (or the appropriate area for the chosen `world_type`) in `Hparams.py`.

## Gazebo worlds

The `labrob_gazebo_worlds/worlds` directory contains worlds for several environments and crowd sizes. Many environments come in two variants:

- **Without `_plugin`** (e.g., `labrob_3rooms_15humans.world`): actor trajectories are fully hand-crafted using the standard Gazebo `<script>` element, with fixed waypoints and timing. Behaviour is deterministic and identical across runs.

- **With `_plugin`** (e.g., `labrob_3rooms_15humans_plugin.world`): actors are controlled by `libEnhancedActorPlugin.so`, which randomizes trajectories at runtime within a per-actor bounding region. The plugin supports several behaviour modes assigned individually to each actor:
  - `static`: actor stands still
  - `static_pair`: two actors stand together (e.g., simulating a conversation)
  - `walking_solo`: single actor walks randomly between waypoints inside its bounding box
  - `walking_pair`: two actors walk together, navigating randomly as a pair

  Parameters such as `velocity`, `num_waypoints`, `min_waypoint_separation`, and `formation_distance` are specified per-actor in the world file. Because waypoints are sampled at runtime, behaviour varies across runs — use `experiment_seed` in `start.launch` for reproducibility.

The `_plugin` variants are generally preferred for performance evaluation as they produce more varied and realistic crowd behaviour.

## Usage

### Option A — Unified launch (recommended for simulation)
A single launch file starts the Gazebo world, spawns the robot, and brings up all navigation nodes:
```bash
roslaunch crowd_navigation_core start.launch world:=WORLD experiment_seed:=SEED
```
where `WORLD` is one of the worlds in the `labrob_gazebo_worlds/worlds` directory (e.g., `labrob_3rooms_15humans_plugin`) and `SEED` is an integer controlling randomization (default: `0`).

> **NOTE:** `world_type` is automatically derived from the `WORLD` argument, so no manual configuration is needed in `Hparams.py`. The robot's initial pose is also automatically sampled from the feasible areas by the `seeded_robot_initializer` node.

### Option B — Manual step-by-step launch
**Step 1.** Set `world_type` in `Hparams.py` to match the environment you intend to use. The `world_type` defines the decomposition of the environment into convex navigable areas, their intersections, and the viapoints used by the planner to navigate across rooms — mismatching it with the actual environment will cause the robot to plan paths in the wrong geometry.

**Step 2.** Start the Gazebo simulation (make sure `simulation=True` in `Hparams.py`):
```bash
roslaunch labrob_tiago_gazebo tiago_gazebo.launch public_sim:=true end_effector:=pal-gripper world:=WORLD
```
where `WORLD` is one of the worlds in the `labrob_gazebo_worlds/worlds` directory.

**Step 3.** Sample a valid robot spawn pose from the navigable areas:
```bash
roslaunch crowd_navigation_core seeded_robot_initializer.launch experiment_seed:=SEED
```
Without this step, the robot spawns at the default pose defined in `tiago_gazebo.launch`, which may be outside the environment or in collision with walls depending on the chosen world. Alternatively, you can manually set a valid pose by editing the `gzpose` default argument in `tiago_gazebo.launch` directly.

**Step 4.** Run all navigation nodes at once:
```bash
roslaunch crowd_navigation_core crowd_navigation.launch
```
This starts the camera perception, laser perception, crowd prediction, and motion generation nodes together.

Alternatively, run the modules individually (each in a separate terminal), depending on the active `perception` mode:
```bash
roslaunch crowd_navigation_core laser_perception.launch    # if perception includes laser
roslaunch crowd_navigation_core camera_perception.launch   # if perception includes camera
roslaunch crowd_navigation_core crowd_prediction.launch
roslaunch crowd_navigation_core motion_generation.launch
```

### Setting the target position

**When using `start.launch` (Option A):** target setting is handled automatically by the `target_sampler` node, which randomly samples a valid goal from the navigable `areas` defined in `Hparams.py`. Its behaviour is controlled by parameters set directly in `start.launch`:
- `min_goal_distance`: minimum distance from the robot's initial pose (default: `10.0` m)
- `goal_clearance`: margin from area boundaries when sampling (default: `0.4` m)
- `goal_sampling_attempts`: number of random attempts before falling back to the area centroid (default: `200`)
- `one_shot`: if `true` (default), only one target is sent; the node stops after the robot reaches it

**When running modules separately (Option B):** two approaches are available:

- *Automated sequence* — define a list of goal positions in the `target_positions` array in `Hparams.py`, then launch:
  ```bash
  roslaunch crowd_navigation_core target_setter.launch
  ```
  The `target_setter` node publishes goals from the list in order, advancing to the next one each time the current target is achieved.

- *Interactive (manual)* — send a single target on demand:
  ```bash
  roslaunch crowd_navigation_core send_desired_target_position.launch x_des:=X y_des:=Y
  ```
  where `X` and `Y` are the coordinates of the desired goal. This can be run multiple times to update the target.

## Logging
When the nodes are shut down, data are saved as `.json` files in `/tmp/crowdnav/data`. The filename is automatically generated from the active configuration in `Hparams.py` using the pattern:
```
<perception>_<world_type>_<n_agents>_<experiment_seed>
```
For example: `both_three_rooms_15_0_generator.json`.

Each run produces several files that share the same base name and differ by suffix:
- `_generator.json` — robot state, control inputs, target, and solver data (always produced);
- `_predictor.json` — crowd predictions (produced when at least one agent is tracked);
- `_laser.json` — laser detections (produced when the perception mode includes the laser, i.e. `LASER` or `BOTH`);
- `_camera.json` — camera detections (produced when the perception mode includes the camera, i.e. `CAMERA` or `BOTH`).

These logs are self-describing and can be inspected or post-processed with your own analysis tools.

## Plotting
A helper node replays a saved run as animations to visualize what happened: the laser and/or camera measurements (depending on the `perception` mode that was used) and the robot motion together with the crowd estimates and predictions.
```bash
roslaunch crowd_navigation_core plotter.launch filename:=FILENAME
```
where `FILENAME` is the base name of the saved `.json` files, **without** the `_generator`/`_predictor` suffix and **without** the `.json` extension (e.g. use `filename:=both_three_rooms_15_0` for `both_three_rooms_15_0_generator.json`).

To also save the animations as `.mp4` files (written to `/tmp/crowdnav/animations`), add `save:=true`:
```bash
roslaunch crowd_navigation_core plotter.launch filename:=FILENAME save:=true
```
Saving requires `ffmpeg` to be installed.

## License
This project is released under the MIT License. See the [LICENSE](LICENSE) file for details.

## Citation
If you use this code in your research, please cite:

> Gravina G, D'Orazio F, Cipriano M, Belvedere T and Oriolo G (2026) Crowd navigation in a multi-room environment: a model predictive control framework for mobile robots. *Front. Robot. AI* 13:1812386. doi: [10.3389/frobt.2026.1812386](https://doi.org/10.3389/frobt.2026.1812386)

```bibtex
@ARTICLE{10.3389/frobt.2026.1812386,
AUTHOR={Gravina, Giovanbattista  and D’Orazio, Francesco  and Cipriano, Michele  and Belvedere, Tommaso  and Oriolo, Giuseppe },   
TITLE={Crowd navigation in a multi-room environment: a model predictive control framework for mobile robots},
JOURNAL={Frontiers in Robotics and AI},
VOLUME={Volume 13 - 2026},
YEAR={2026},
URL={https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2026.1812386},
DOI={10.3389/frobt.2026.1812386}}
```
