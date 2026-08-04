import numpy as np
import rospy
import math
import json
import os
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge
from matplotlib.patches import Polygon as Area
from matplotlib.animation import FuncAnimation

from crowd_navigation_core.utils import *
from crowd_navigation_core.Hparams import *

class Plotter:
    '''
    Replay the logged data as animations to visualize what happened during a run:
      - laser measurements (if the laser was used),
      - camera measurements (if the camera was used),
      - robot motion, together with the crowd estimates/predictions.

    The animations are shown on screen and, when save=True, saved as .mp4 files.
    '''
    def __init__(self, filename, save):
        self.filename = filename
        self.save = save
        self.still_wheel_threshold = 0.02
        # 'seaborn-whitegrid' was renamed in newer matplotlib; fall back gracefully.
        try:
            plt.style.use('seaborn-whitegrid')
        except OSError:
            pass

        # Specify logging directory
        log_dir = Hparams.log_dir
        if not os.path.exists(log_dir):
            raise Exception(f"Specified directory not found")

        # Set the loggers
        self.log_generator = os.path.join(log_dir, filename + '_generator.json')
        self.log_predictor = os.path.join(log_dir, filename + '_predictor.json')
        self.log_laser = os.path.join(log_dir, filename + '_laser.json')
        self.log_camera = os.path.join(log_dir, filename + '_camera.json')

        # Extract the generator dictionary
        if os.path.exists(self.log_generator):
            with open(self.log_generator, 'r') as file:
                self.generator_dict = json.load(file)
                self.perception_mode = self.generator_dict['perception']
                self.n_filters = self.generator_dict['n_filters']
        else:
            raise Exception(
                f"Generator logfile not found"
            )

        # Extract the predictor dictionary
        if self.n_filters > 0:
            if os.path.exists(self.log_predictor):
                with open(self.log_predictor, 'r') as file:
                    self.predictor_dict = json.load(file)
            else:
                raise Exception(
                    f"Predictor logfile not found"
                )

            if self.perception_mode in ('Perception.BOTH', 'Perception.CAMERA'):
                # Extract the camera detector dictionary
                if os.path.exists(self.log_camera):
                    with open(self.log_camera, 'r') as file:
                        self.camera_dict = json.load(file)
                else:
                    raise Exception(
                        f"Camera detector logfile not found"
                    )

            if self.perception_mode in ('Perception.BOTH', 'Perception.LASER'):
                # Extract the laser detector dictionary
                if os.path.exists(self.log_laser):
                    with open(self.log_laser, 'r') as file:
                        self.laser_dict = json.load(file)
                else:
                    raise Exception(
                        f"Laser detector logfile not found"
                    )

        # Specify saving animations directory
        self.animation_dir = '/tmp/crowdnav/animations'
        if not os.path.exists(self.animation_dir):
            os.makedirs(self.animation_dir)

        # The full logged time-series is replayed. These indices are kept so the
        # data-access helpers below stay simple to extend if partial replay is needed.
        self.slice_start = 0
        self.slice_end = None

    def _slice_data(self, data):
        if data is None:
            return data
        return data[self.slice_start:self.slice_end]

    def _slice_array(self, data, dtype=None):
        sliced = self._slice_data(data)
        if dtype is None:
            return np.asarray(sliced)
        return np.asarray(sliced, dtype=dtype)

    def _plot_areas(self, ax, areas, viapoints, alpha=0.4):
        color = 'r'
        area_patches = []
        for i, vertexes in enumerate(areas):
            area = Area(vertexes,
                        closed=True,
                        fill=True,
                        facecolor=color,
                        alpha=alpha,
                        edgecolor=color,
                        linestyle='--',
                        label=f'Area {i}')
            ax.add_patch(area)
            area_patches.append(area)
        if len(viapoints) > 0:
            ax.scatter(viapoints[:, 0], viapoints[:, 1], marker='o', color='blue', alpha=0.2)
        return area_patches

    def _plot_walls(self, ax, walls):
        for wall_start, wall_end in walls:
            ax.plot([wall_start[0], wall_end[0]],
                    [wall_start[1], wall_end[1]],
                    'k-',
                    linewidth=4)

    def _plot_laser_fov(self, fov, theta, laser_pos, range_min, range_max, angle_min, angle_max):
        fov.set_center(laser_pos)
        fov.set_radius(range_max)
        fov.set_theta1((theta + angle_min) * 180 / np.pi)
        fov.set_theta2((theta + angle_max) * 180 / np.pi)
        fov.set_width(range_max - range_min)

    def _plot_camera_fov(self, fov, cam_pos, cam_angle, cam_horz_fov, min_length, max_length):
        vertexes = np.zeros((4, 2))
        min_angle = cam_angle - cam_horz_fov / 2
        max_angle = cam_angle + cam_horz_fov / 2
        vertexes[0, :] = cam_pos + np.array([math.cos(min_angle), math.sin(min_angle)]) * min_length
        vertexes[1] = cam_pos + np.array([math.cos(min_angle), math.sin(min_angle)]) * max_length
        vertexes[2] = cam_pos + np.array([math.cos(max_angle), math.sin(max_angle)]) * max_length
        vertexes[3] = cam_pos + np.array([math.cos(max_angle), math.sin(max_angle)]) * min_length
        fov.set_xy(vertexes)

    def _set_axis_properties(self,
                             ax,
                             xlabel,
                             ylabel,
                             title=None,
                             set_aspect=False,
                             grid=True,
                             legend=False,
                             xlim=None,
                             ylim=None):
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        if set_aspect:
            ax.set_aspect('equal', adjustable='box')
        ax.grid(grid)
        if legend:
            ax.legend(loc='lower left')
        if xlim is not None:
            ax.set_xlim(xlim)
        if ylim is not None:
            ax.set_ylim(ylim)

    def plot_camera(self):
        # Specify the saving path
        cam_path = os.path.join(self.animation_dir, self.filename + '_camera.mp4')

        # Extract the camera detector data
        time = self._slice_array(self.camera_dict['cpu_time'])[:, 1]
        time -= time[0]
        measurements = self._slice_data(self.camera_dict['measurements'])
        robot_config = self._slice_array(self.camera_dict['robot_config'])
        camera_pos = self._slice_array(self.camera_dict['camera_position'])
        camera_angle = - self._slice_array(self.camera_dict['camera_horz_angle']) - np.pi / 2
        b = self.camera_dict['b']
        shooting_nodes = robot_config.shape[0]
        robot_center = np.empty((shooting_nodes, 2))
        for i in range(shooting_nodes):
            robot_center[i, 0] = robot_config[i, 0] - b * math.cos(robot_config[i, 2])
            robot_center[i, 1] = robot_config[i, 1] - b * math.sin(robot_config[i, 2])

        frequency = self.camera_dict['frequency']
        base_radius = self.camera_dict['base_radius']
        simulation = self.camera_dict['simulation']
        if simulation:
            n_agents = self.camera_dict['n_agents']
            agents_pos = self._slice_array(self.camera_dict['agents_pos'])
            agent_radius = self.camera_dict['agent_radius']
        cam_horz_fov = self.camera_dict['horz_fov']
        range_min = self.camera_dict['min_range']
        range_max = self.camera_dict['max_range']
        min_fov_length = range_min / math.cos(cam_horz_fov / 2)
        max_fov_length = range_max / math.cos(cam_horz_fov / 2)
        areas = self.generator_dict['areas']
        viapoints = []
        walls = self.generator_dict['walls']

        # Plot animation with camera measurements
        fig, ax = plt.subplots(figsize=(8, 8))

        robot = Circle(np.zeros(2), np.zeros(1), facecolor='none', edgecolor='k', label='TIAGo')
        controlled_pt = ax.scatter([], [], marker='.', color='k')
        robot_label = ax.text(np.nan, np.nan, robot.get_label(), fontsize=16, ha='left', va='bottom')
        meas, = ax.plot([], [], color='blue', marker='.', markersize=5, linestyle='', label='meas')
        fov = Area(np.full((1, 2), np.nan), closed=True, fill=True, facecolor='purple', alpha=0.1)

        if simulation:
            agents = []
            agents_label = []
            agents_clearance = []
            for i in range(n_agents):
                agents.append(ax.scatter([], [], marker='.', label='ag{}'.format(i+1), color='k', alpha=0.3))
                agents_clearance.append(Circle(np.zeros(2), np.zeros(1), facecolor='none', edgecolor='k', linestyle='--', alpha=0.3))
                agents_label.append(ax.text(np.nan, np.nan, agents[i].get_label(), fontsize=16, ha='left', va='bottom', alpha=0.3))

        self._set_axis_properties(ax,
                                  xlabel="$x \quad [m]$",
                                  ylabel="$y \quad [m]$",
                                  title='TIAGo camera measurements',
                                  grid=False,
                                  set_aspect=True)
        if simulation:
            for i in range(n_agents):
                agents_label[i].set_visible(False)

        # init and update function for the camera animation
        def init():
            self._plot_walls(ax, walls)
            self._plot_areas(ax, areas, viapoints, alpha=0.1)
            robot.set_center(robot_center[0])
            robot.set_radius(base_radius)
            ax.add_patch(robot)
            ax.add_patch(fov)
            controlled_pt.set_offsets(robot_config[0, :2])
            robot_label.set_position(robot_center[0])
            if simulation:
                for i in range(n_agents):
                    agent_pos = agents_pos[0, i, :]
                    agents[i].set_offsets(agent_pos)
                    agents_clearance[i].set_center(agent_pos)
                    agents_clearance[i].set_radius(agent_radius)
                    ax.add_patch(agents_clearance[i])
                    agents_label[i].set_position(agent_pos)
                return robot, robot_label, controlled_pt, agents, agents_clearance, agents_label
            return robot, robot_label, controlled_pt

        def update(frame):
            if frame == shooting_nodes - 1:
                animation.event_source.stop()
            ax.set_title(f'TIAGo camera measurements, t={time[frame]:.2f}')
            robot.set_center(robot_center[frame])
            controlled_pt.set_offsets(robot_config[frame, :2])
            robot_label.set_position(robot_center[frame])
            current_meas = np.array(measurements[frame])
            self._plot_camera_fov(fov,
                                  camera_pos[frame],
                                  -camera_angle[frame],
                                  cam_horz_fov,
                                  min_fov_length,
                                  max_fov_length)
            if current_meas.shape[0] > 0:
                meas.set_data(current_meas[:, 0], current_meas[:, 1])
            else:
                meas.set_data([], [])
            if simulation:
                for i in range(n_agents):
                    agent_pos = agents_pos[frame, i, :]
                    agents[i].set_offsets(agent_pos)
                    agents_clearance[i].set_center(agent_pos)
                    agents_label[i].set_position(agent_pos)
                return robot, robot_label, controlled_pt, meas, \
                       agents, agents_clearance, agents_label
            return robot, robot_label, controlled_pt, meas

        animation = FuncAnimation(fig, update,
                                  frames=shooting_nodes,
                                  init_func=init,
                                  interval=1/frequency,
                                  blit=False,
                                  repeat=False)

        if self.save:
            animation.save(cam_path, writer='ffmpeg', fps=frequency, dpi=80)
            print("Camera animation saved")

        plt.show()

    def plot_laser(self):
        # Specify the saving path
        las_path = os.path.join(self.animation_dir, self.filename + '_laser.mp4')

        # Extract the laser detector data
        time = self._slice_array(self.laser_dict['cpu_time'])[:, 1]
        time -= time[0]
        laser_scans = self._slice_data(self.laser_dict['laser_scans'])
        measurements = self._slice_data(self.laser_dict['measurements'])
        robot_config = self._slice_array(self.laser_dict['robot_config'])
        laser_pos = self._slice_array(self.laser_dict['laser_position'])
        b = self.laser_dict['b']
        shooting_nodes = robot_config.shape[0]
        robot_center = np.empty((shooting_nodes, 2))
        for i in range(shooting_nodes):
            robot_center[i, 0] = robot_config[i, 0] - b * math.cos(robot_config[i, 2])
            robot_center[i, 1] = robot_config[i, 1] - b * math.sin(robot_config[i, 2])

        frequency = self.laser_dict['frequency']
        base_radius = self.laser_dict['base_radius']
        simulation = self.laser_dict['simulation']
        if simulation:
            n_agents = self.laser_dict['n_agents']
            agents_pos = self._slice_array(self.laser_dict['agents_pos'])
            agent_radius = self.laser_dict['agent_radius']
        angle_inc = self.laser_dict['angle_inc']
        laser_offset = self.laser_dict['laser_offset']
        angle_min = self.laser_dict['angle_min'] + angle_inc * laser_offset
        angle_max = self.laser_dict['angle_max'] - angle_inc * laser_offset
        range_min = self.laser_dict['range_min']
        range_max = self.laser_dict['range_max']
        areas = self.generator_dict['areas']
        viapoints = []
        walls = self.generator_dict['walls']

        # Plot animation with laser measurements
        fig, ax = plt.subplots(figsize=(8, 8))

        robot = Circle(np.zeros(2), np.zeros(1), facecolor='none', edgecolor='k', label='$r$')
        controlled_pt = ax.scatter([], [], marker='.', color='k')
        robot_label = ax.text(np.nan, np.nan, robot.get_label(), fontsize=16, ha='left', va='bottom')
        scans, = ax.plot([], [], color='magenta', marker='.', markersize=2, linestyle='', label='scans')
        meas, = ax.plot([], [], color='b', marker='x', markersize=5, linestyle='', label='meas')
        fov = Wedge(np.zeros(1), np.zeros(1), 0.0, 0.0, color='cyan', alpha=0.1)

        if simulation:
            agents = []
            agents_label = []
            agents_clearance = []
            for i in range(n_agents):
                agents.append(ax.scatter([], [], marker='.', label='ag{}'.format(i+1), color='k', alpha=0.3))
                agents_clearance.append(Circle(np.zeros(2), np.zeros(1), facecolor='none', edgecolor='k', linestyle='--', alpha=0.3))
                agents_label.append(ax.text(np.nan, np.nan, agents[i].get_label(), fontsize=16, ha='left', va='bottom', alpha=0.3))

        self._set_axis_properties(ax,
                                  xlabel="$x \quad [m]$",
                                  ylabel="$y \quad [m]$",
                                  title='TIAGo laser measurements',
                                  grid=False,
                                  set_aspect=True)
        if simulation:
            for i in range(n_agents):
                agents_label[i].set_visible(False)

        # Switch off fovs
        if self.n_filters > 0:
            fov.set_visible(False)

        # init and update function for the laser animation
        def init():
            self._plot_walls(ax, walls)
            self._plot_areas(ax, areas, viapoints, alpha=0.1)
            robot.set_center(robot_center[0])
            robot.set_radius(base_radius)
            ax.add_patch(robot)
            ax.add_patch(fov)
            controlled_pt.set_offsets(robot_config[0, :2])
            robot_label.set_position(robot_center[0])
            if simulation:
                for i in range(n_agents):
                    agent_pos = agents_pos[0, i, :]
                    agents[i].set_offsets(agent_pos)
                    agents_clearance[i].set_center(agent_pos)
                    agents_clearance[i].set_radius(agent_radius)
                    ax.add_patch(agents_clearance[i])
                    agents_label[i].set_position(agent_pos)
                return robot, robot_label, controlled_pt, agents, agents_clearance, agents_label
            return robot, robot_label, controlled_pt

        def update(frame):
            if frame == shooting_nodes - 1:
                animation.event_source.stop()
            ax.set_title(f'TIAGo laser measurements, t={time[frame]:.2f}')
            robot.set_center(robot_center[frame])
            controlled_pt.set_offsets(robot_config[frame, :2])
            robot_label.set_position(robot_center[frame])
            current_meas = np.array(measurements[frame])
            current_scans = np.array(laser_scans[frame])
            self._plot_laser_fov(fov,
                                 robot_config[frame, 2],
                                 laser_pos[frame],
                                 range_min,
                                 range_max,
                                 angle_min,
                                 angle_max)
            if current_scans.shape[0] > 0:
                scans.set_data(current_scans[:, 0], current_scans[:, 1])
            else:
                scans.set_data([], [])
            if current_meas.shape[0] > 0:
                meas.set_data(current_meas[:, 0], current_meas[:, 1])
            else:
                meas.set_data([], [])
            if simulation:
                for i in range(n_agents):
                    agent_pos = agents_pos[frame, i, :]
                    agents[i].set_offsets(agent_pos)
                    agents_clearance[i].set_center(agent_pos)
                    agents_label[i].set_position(agent_pos)
                return robot, robot_label, controlled_pt, meas, \
                       agents, agents_clearance, agents_label
            return robot, robot_label, controlled_pt, meas

        animation = FuncAnimation(fig, update,
                                  frames=shooting_nodes,
                                  init_func=init,
                                  interval=1/frequency,
                                  blit=False,
                                  repeat=False)

        if self.save:
            animation.save(las_path, writer='ffmpeg', fps=frequency, dpi=80)
            print("Laser animation saved")

        plt.show()

    def plot_motion(self):
        motion_savepath = os.path.join(self.animation_dir, self.filename + '_motion.mp4')

        # Extract the generator data
        cpu_time = self._slice_array(self.generator_dict['cpu_time'])
        iteration_durations = cpu_time[:, 0].copy()
        time = cpu_time[:, 1].copy()
        t0 = time[0]
        robot_states = self._slice_array(self.generator_dict['robot_state'])[0:, :]
        configurations = robot_states[:, :3]
        b = self.generator_dict['b']
        robot_center = np.empty((configurations.shape[0], 2))
        for i in range(configurations.shape[0]):
            robot_center[i, 0] = configurations[i, 0] - b * math.cos(configurations[i, 2])
            robot_center[i, 1] = configurations[i, 1] - b * math.sin(configurations[i, 2])
        robot_predictions = self._slice_array(self.generator_dict['robot_predictions'])[0:, :]

        wheels_velocities = self._slice_array(self.generator_dict['wheels_velocities'])[0:, :]
        targets = self._slice_array(self.generator_dict['targets'])[0:, :]
        target_viapoints = self._slice_array(self.generator_dict['target_viapoints'])[0:, :]
        area_index = self._slice_array(self.generator_dict['area_index'])[0:]
        inputs = self._slice_array(self.generator_dict['inputs'])[0:, :]
        areas = self.generator_dict['areas']
        viapoints = np.array(self.generator_dict['viapoints'])
        walls = self.generator_dict['walls']

        if self.n_filters > 0:
            agents_predictions = self._slice_array(self.generator_dict['agents_predictions'])[0:, :]
        simulation = self.generator_dict['simulation']
        if simulation:
            n_agents = self.generator_dict['n_agents']
            agents_pos = self._slice_array(self.generator_dict['agents_pos'])[0:, :]
        agent_radius = self.generator_dict['agent_radius']
        frequency = self.generator_dict['frequency']
        base_radius = self.generator_dict['base_radius']
        dt = self.generator_dict['dt']
        N_horizon = self.generator_dict['N_horizon']
        laser_rel_pos = np.array(self.generator_dict['laser_rel_pos'])
        shooting_nodes = inputs.shape[0]

        failure_highlight_window = 1.5
        failure_windows = []
        failure_events = self.generator_dict.get('failure_events', [])
        if len(failure_events) > 0:
            for event in failure_events:
                if event.get('type') != 'mpc_failure':
                    continue
                event_time = event.get('time')
                if event_time is None:
                    continue
                rel_start = event_time - t0
                failure_windows.append((rel_start, rel_start + failure_highlight_window))

        collision_threshold = base_radius + agent_radius
        min_collision_distances = None
        if simulation and 'agents_pos' in self.generator_dict:
            if agents_pos.size > 0 and agents_pos.shape[1] > 0:
                valid_len = min(agents_pos.shape[0], robot_center.shape[0])
                diffs = agents_pos[:valid_len] - robot_center[:valid_len, None, :]
                distances = np.linalg.norm(diffs, axis=2)
                min_collision_distances = np.min(distances, axis=1)

        first_move_idx = None
        if wheels_velocities.size > 0 and wheels_velocities.ndim == 2 and wheels_velocities.shape[1] >= 2:
            align_len = min(wheels_velocities.shape[0], configurations.shape[0])
            if align_len > 0:
                wheel_abs = np.maximum(np.abs(wheels_velocities[:align_len, 0]),
                                       np.abs(wheels_velocities[:align_len, 1]))
                still_flags = np.zeros(configurations.shape[0], dtype=bool)
                still_flags[:align_len] = wheel_abs < self.still_wheel_threshold
                moving_indices = np.where(~still_flags)[0]
                if moving_indices.size > 0:
                    first_move_idx = int(moving_indices[0])

        goal_reached_idx = None
        dists_to_final_goal = np.linalg.norm(configurations[:, :2] - targets[-1, :2][None, :], axis=1)
        goal_arrival = np.where(dists_to_final_goal < 0.5)[0]
        if goal_arrival.size > 0:
            goal_reached_idx = int(goal_arrival[0])

        if self.n_filters > 0:
            if self.perception_mode in ('Perception.LASER', 'Perception.BOTH'):
                laser_range_max = self.laser_dict['range_max']
                laser_range_min = self.laser_dict['range_min']
                angle_inc = self.laser_dict['angle_inc']
                laser_offset = self.laser_dict['laser_offset']
                angle_min = self.laser_dict['angle_min'] + angle_inc * laser_offset
                angle_max = self.laser_dict['angle_max'] - angle_inc * laser_offset

            if self.perception_mode in ('Perception.CAMERA', 'Perception.BOTH'):
                camera_range_max = self.camera_dict['max_range']
                camera_range_min = self.camera_dict['min_range']
                cam_horz_fov = self.camera_dict['horz_fov']
                min_fov_length = camera_range_min / math.cos(cam_horz_fov / 2)
                max_fov_length = camera_range_max / math.cos(cam_horz_fov / 2)
                camera_pos = self._slice_array(self.generator_dict['camera_position'])[0:, :]
                camera_angle = - self._slice_array(self.generator_dict['camera_horz_angle'])[0:] - np.pi / 2

        # Figure to plot motion animation
        fig, ax = plt.subplots(figsize=(8, 8))
        robot = Circle(np.zeros(2), np.zeros(1), facecolor='none', edgecolor='k', label=r"$r$")
        controlled_pt = ax.scatter([], [], marker='.', color='k')
        robot_label = ax.text(np.nan, np.nan, robot.get_label(), fontsize=16, ha='left', va='bottom')
        target_viapoint = ax.scatter([], [], marker='o', color='magenta', alpha=0.5)
        if self.n_filters > 0:
            if simulation:
                agents = []
                agents_label = []
                agents_clearance = []
                for i in range(n_agents):
                    agents.append(ax.scatter([], [], marker='.', label='ag{}'.format(i+1), color='k', alpha=0.1))
                    agents_clearance.append(Circle(np.zeros(2), np.zeros(1), facecolor='none', edgecolor='k', linestyle='-', alpha=0.3))
                    agents_label.append(ax.text(np.nan, np.nan, agents[i].get_label(), fontsize=16, ha='left', va='bottom', alpha=0.3))
            if self.perception_mode in ('Perception.LASER', 'Perception.BOTH'):
                laser_fov = Wedge(np.zeros(1), np.zeros(1), 0.0, 0.0, color='cyan', alpha=0.1)
            if self.perception_mode in ('Perception.CAMERA', 'Perception.BOTH'):
                camera_fov = Area(np.full((1, 2), np.nan), closed=True, fill=True, facecolor='purple', alpha=0.1)
            estimates = []
            estimates_label = []
            estimates_clearance = []
            for i in range(self.n_filters):
                estimate_label = rf'$h_{{{i+1}}}$'
                estimates.append(ax.scatter([], [], marker='.', label=estimate_label, color='red'))
                estimates_clearance.append(Circle(np.zeros(2), np.zeros(1), facecolor='none', edgecolor='red', linestyle='-'))
                estimates_label.append(ax.text(np.nan, np.nan, estimates[i].get_label(), fontsize=16, ha='left', va='bottom'))

            # Orange segment showing each tracked agent's predicted motion over the horizon.
            agents_pred_lines = []
            for i in range(self.n_filters):
                pred_line, = ax.plot([], [], color='orange', linestyle='-', linewidth=2, alpha=0.8,
                                     label='agent prediction' if i == 0 else None)
                agents_pred_lines.append(pred_line)

        traj_line, = ax.plot([], [], color='blue', linestyle='--', label='trajectory')
        robot_pred_line, = ax.plot([], [], color='green', label='prediction')
        failure_indicator = ax.text(0.5,
            0.95,
            'MPC failure',
                transform=ax.transAxes,
                color='orange',
                fontsize=14,
                fontweight='bold',
                bbox=dict(facecolor='white', alpha=0.8, edgecolor='orange'),
                ha='center',
                visible=False)
        collision_indicator = ax.text(0.98,
            0.95,
            'collision',
            transform=ax.transAxes,
            color='red',
            fontsize=14,
            fontweight='bold',
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='red'),
            ha='right',
            visible=False)

        self._set_axis_properties(ax,
                                  xlabel="$x \quad [m]$",
                                  ylabel="$y \quad [m]$",
                                  title='TIAGo motion',
                                  grid=False,
                                  set_aspect=True)

        self._plot_walls(ax, walls)
        area_patches = self._plot_areas(ax, areas, viapoints)

        if simulation and self.n_filters > 0:
            for i in range(n_agents):
                agents_label[i].set_visible(False)

        # Switch off fovs
        if self.n_filters > 0:
            if self.perception_mode in ('Perception.LASER', 'Perception.BOTH'):
                laser_fov.set_visible(False)
            if self.perception_mode in ('Perception.CAMERA', 'Perception.BOTH'):
                camera_fov.set_visible(False)

        def is_failure_active(frame_start, frame_end):
            for start, end in failure_windows:
                if frame_start <= end and frame_end >= start:
                    return True
            return False

        def is_collision_active(frame_idx):
            if min_collision_distances is None:
                return False
            if frame_idx >= len(min_collision_distances):
                return False
            return min_collision_distances[frame_idx] < collision_threshold

        # No adaptive window: matplotlib autoscales to the plotted walls, so the whole
        # environment stays in frame for the entire animation (original behavior).

        # init and update function for the motion animation
        def init_motion():
            # One-time setup only: fix the artist radii and register every patch with
            # the axes. The frame-0 state itself is produced by update_motion(0) below,
            # so the first frame is identical to every other frame (no stale first frame).
            robot.set_radius(base_radius)
            ax.add_patch(robot)
            if self.n_filters > 0:
                for i in range(self.n_filters):
                    ax.add_patch(estimates_clearance[i])
                if self.perception_mode in ('Perception.LASER', 'Perception.BOTH'):
                    ax.add_patch(laser_fov)
                if self.perception_mode in ('Perception.CAMERA', 'Perception.BOTH'):
                    ax.add_patch(camera_fov)
                if simulation:
                    for i in range(n_agents):
                        agents_clearance[i].set_radius(agent_radius)
                        ax.add_patch(agents_clearance[i])
            return update_motion(0)

        def update_motion(frame):
            if frame == shooting_nodes - 1:
                motion_animation.event_source.stop()

            ref_frame = min(frame, goal_reached_idx) if goal_reached_idx is not None else frame
            travel_t = time[ref_frame] - time[first_move_idx] if first_move_idx is not None and frame >= first_move_idx else 0.0
            t_rel = time[frame] - time[0]
            if min_collision_distances is not None and frame < len(min_collision_distances):
                dist_str = f'd_min={min_collision_distances[frame]:.2f}m'
            else:
                dist_str = 'd_min=N/A'
            ax.set_title(f'TIAGo motion | t={t_rel:.2f}s | travel={travel_t:.1f}s | {dist_str}')
            robot_prediction = robot_predictions[frame, :, :]
            current_target = targets[frame, :]
            current_viapoint = target_viapoints[frame]
            traj_line.set_data(configurations[:frame + 1, 0], configurations[:frame + 1, 1])
            robot_pred_line.set_data(robot_prediction[:, 0], robot_prediction[:, 1])

            robot.set_center(robot_center[frame])
            controlled_pt.set_offsets(configurations[frame, :2])
            robot_label.set_position(robot_center[frame])
            frame_start = time[frame]
            frame_duration = iteration_durations[frame] if frame < len(iteration_durations) else 0.0
            frame_end = frame_start + frame_duration
            failure_active = is_failure_active(frame_start, frame_end)
            collision_active = is_collision_active(frame)

            if collision_active:
                robot.set_edgecolor('red')
            elif failure_active:
                robot.set_edgecolor('orange')
            else:
                robot.set_edgecolor('k')

            failure_indicator.set_visible(failure_active)
            collision_indicator.set_visible(collision_active)
            if current_viapoint[0] != current_target[0] or current_viapoint[1] != current_target[1]:
                target_viapoint.set_visible(True)
                target_viapoint.set_offsets(target_viapoints[frame])
            else:
                target_viapoint.set_visible(False)
            # Highlight the region the robot is currently in; dim the others.
            area_patches[area_index[frame]].set_alpha(0.2)
            for patch in area_patches[:area_index[frame]]:
                patch.set_alpha(0.05)
            for patch in area_patches[area_index[frame] + 1:]:
                patch.set_alpha(0.05)
            if self.n_filters > 0:
                for i in range(self.n_filters):
                    est = agents_predictions[frame, i, :4]
                    if est[0] != Hparams.nullpos and est[1] != Hparams.nullpos:
                        p0 = est[:2]
                        v0 = est[2:4]
                        estimates[i].set_offsets(p0)
                        estimates[i].set_visible(True)
                        estimates_clearance[i].set_center(p0)
                        estimates_clearance[i].set_radius(agents_predictions[frame, i, 4])
                        estimates_label[i].set_position(p0)
                        estimates_label[i].set_visible(True)
                        # Predicted motion over the horizon (constant-velocity rollout),
                        # as in the original implementation.
                        agent_prediction = np.vstack((p0 + v0 * dt,
                                                      p0 + v0 * dt * (N_horizon + 1)))
                        agents_pred_lines[i].set_data(agent_prediction[:, 0], agent_prediction[:, 1])
                        agents_pred_lines[i].set_visible(True)
                    else:
                        estimates[i].set_visible(False)
                        estimates_clearance[i].set_radius(0.0)
                        estimates_label[i].set_visible(False)
                        agents_pred_lines[i].set_visible(False)
                if self.perception_mode in ('Perception.LASER', 'Perception.BOTH'):
                    current_theta = configurations[frame, 2]
                    current_laser_pos = configurations[frame, :2] + z_rotation(current_theta, laser_rel_pos)
                    self._plot_laser_fov(laser_fov,
                                         current_theta,
                                         current_laser_pos,
                                         laser_range_min,
                                         laser_range_max,
                                         angle_min,
                                         angle_max)
                if self.perception_mode in ('Perception.CAMERA', 'Perception.BOTH'):
                    self._plot_camera_fov(camera_fov,
                                          camera_pos[frame],
                                          -camera_angle[frame],
                                          cam_horz_fov,
                                          min_fov_length,
                                          max_fov_length)
                if simulation:
                    for i in range(n_agents):
                        agent_pos = agents_pos[frame, i, :]
                        agents[i].set_offsets(agent_pos)
                        agents_clearance[i].set_center(agent_pos)
                        agents_label[i].set_position(agent_pos)
                    return robot, robot_label, target_viapoint, area_patches, \
                            traj_line, robot_pred_line, agents_pred_lines, \
                            agents, agents_clearance, agents_label, \
                            estimates, estimates_clearance, estimates_label
                return robot, robot_label, target_viapoint, area_patches, \
                        traj_line, robot_pred_line, agents_pred_lines, \
                        estimates, estimates_clearance, estimates_label
            return robot, robot_label, target_viapoint, area_patches, \
                traj_line, robot_pred_line

        motion_animation = FuncAnimation(fig, update_motion,
                                        frames=shooting_nodes,
                                        init_func=init_motion,
                                        blit=False,
                                        interval=1/frequency*500,
                                        repeat=False)

        if self.save:
            motion_animation.save(motion_savepath, writer='ffmpeg', fps=frequency, dpi=80)
            print("Motion animation saved")

        plt.show()

    def run(self):
        if self.n_filters > 0:
            if self.perception_mode == 'Perception.BOTH':
                self.plot_laser()
                self.plot_camera()
            elif self.perception_mode == 'Perception.LASER':
                self.plot_laser()
            elif self.perception_mode == 'Perception.CAMERA':
                self.plot_camera()
        self.plot_motion()

def main():
    rospy.init_node('tiago_plotter', anonymous=True, log_level=rospy.INFO)
    rospy.loginfo('TIAGo plotter module [OK]')

    filename = rospy.get_param('/filename')
    save = rospy.get_param('/save', False)
    plotter = Plotter(filename, save)
    plotter.run()

if __name__ == '__main__':
    main()
