import math
import random
from typing import Dict, List

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Point, Quaternion, Twist
from shapely.geometry import MultiPolygon, Point as ShapelyPoint, Polygon
from tf.transformations import quaternion_from_euler

from crowd_navigation_core.Hparams import Hparams


class SeededRobotInitializer:
    """Sample a reproducible pose using Hparams areas and push it to Gazebo."""

    def __init__(self) -> None:
        self.robot_name = rospy.get_param("~robot_name", "tiago")
        self.world_name = rospy.get_param(
            "~world",
            rospy.get_param("/crowd_navigation_core/world", "labrob_3rooms_15humans_plugin"),
        )
        base_seed = int(
            rospy.get_param(
                "~experiment_seed",
                rospy.get_param("/crowd_navigation_core/experiment_seed", 0),
            )
        )
        seed_offset = int(rospy.get_param("~pose_seed_offset", 0))
        self.seed = base_seed + seed_offset
        self.wait_timeout = float(rospy.get_param("~spawn_timeout", 30.0))
        self.hparams = Hparams()
        self.areas = getattr(self.hparams, "areas", [])
        if not self.areas:
            raise rospy.ROSException("Hparams.areas is empty; cannot sample spawn pose")
        self.spawn_area_indexes = self._resolve_spawn_area_indexes()
        self.clearance = float(rospy.get_param("~spawn_clearance", 1.0))
        self._polygons = self._build_polygons(self.spawn_area_indexes)

        self.pose = self._sample_pose()
        rospy.set_param("/crowd_navigation_core/initial_pose", self.pose)

    def _resolve_spawn_area_indexes(self) -> List[int]:
        param_key = "~spawn_area_indexes"
        if rospy.has_param(param_key):
            indices = rospy.get_param(param_key)
            if isinstance(indices, (list, tuple)):
                parsed = [int(idx) for idx in indices]
            else:
                parsed = [int(indices)]
            return parsed

        available = list(range(len(self.areas)))
        if not available:
            raise rospy.ROSException("No spawn areas available in Hparams")
        rospy.loginfo(
            "No spawn_area_indexes specified; using all %d areas by default", len(available)
        )
        return available

    def _build_polygons(self, area_indexes: List[int]) -> Dict[int, Polygon]:
        polygons: Dict[int, Polygon] = {}
        for idx in area_indexes:
            try:
                area = np.asarray(self.areas[idx])
            except IndexError as exc:
                raise rospy.ROSException(
                    f"Spawn area index {idx} is invalid for current Hparams"
                ) from exc
            polygon = Polygon(area.tolist())
            shrunk = self._shrink_polygon(polygon)
            if shrunk.is_empty:
                rospy.logwarn(
                    "Spawn clearance %.2fm eliminated area %d; using original polygon",
                    self.clearance,
                    idx,
                )
                shrunk = polygon
            polygons[idx] = shrunk
        return polygons

    def _shrink_polygon(self, polygon: Polygon) -> Polygon:
        if self.clearance <= 0.0:
            return polygon
        buffered = polygon.buffer(-self.clearance)
        if buffered.is_empty:
            return Polygon()
        if isinstance(buffered, Polygon):
            return buffered
        if isinstance(buffered, MultiPolygon):
            largest = max(buffered.geoms, key=lambda geom: geom.area, default=None)
            return largest if largest is not None else Polygon()
        if hasattr(buffered, "geoms"):
            polygons = [geom for geom in buffered.geoms if isinstance(geom, Polygon)]
            if polygons:
                return max(polygons, key=lambda geom: geom.area)
        return Polygon()

    def _sample_pose(self) -> Dict[str, float]:
        rng = random.Random(self.seed)
        area_idx = rng.choice(self.spawn_area_indexes)
        polygon = self._polygons[area_idx]
        x, y = self._sample_point_in_polygon(rng, polygon)
        yaw_bounds = self._yaw_bounds()
        yaw = rng.uniform(*yaw_bounds)
        return {"x": x, "y": y, "yaw": yaw, "area_index": area_idx}

    @staticmethod
    def _sample_point_in_polygon(rng: random.Random, polygon: Polygon) -> List[float]:
        min_x, min_y, max_x, max_y = polygon.bounds
        for _ in range(1000):
            x = rng.uniform(min_x, max_x)
            y = rng.uniform(min_y, max_y)
            if polygon.contains(ShapelyPoint(x, y)):
                return [x, y]
        centroid = polygon.centroid
        rospy.logwarn("Fell back to polygon centroid for spawn sampling")
        return [centroid.x, centroid.y]

    @staticmethod
    def _yaw_bounds() -> List[float]:
        yaw_param = rospy.get_param("~yaw_bounds", None)
        if yaw_param is None:
            return [-math.pi, math.pi]
        if isinstance(yaw_param, (list, tuple)) and len(yaw_param) == 2:
            return [float(yaw_param[0]), float(yaw_param[1])]
        raise rospy.ROSException("~yaw_bounds must be a list of two values [min, max]")

    def run(self) -> None:
        rospy.loginfo(
            "[%s] Waiting for Gazebo services to reposition robot", self.robot_name
        )
        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/set_model_state")
        get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        start_time = rospy.Time.now()
        last_pose = None
        while not rospy.is_shutdown():
            try:
                response = get_model_state(self.robot_name, "")
                if response.success:
                    last_pose = response.pose
                    break
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(5.0, "Waiting for %s: %s", self.robot_name, exc)
            if (rospy.Time.now() - start_time).to_sec() > self.wait_timeout:
                rospy.logwarn(
                    "Robot %s did not appear within %.1fs, proceeding with default height",
                    self.robot_name,
                    self.wait_timeout,
                )
                break
            rospy.sleep(0.5)

        quat = quaternion_from_euler(0.0, 0.0, self.pose["yaw"])
        orientation = Quaternion(*quat)
        state = ModelState()
        state.model_name = self.robot_name
        state.pose.position = Point(self.pose["x"], self.pose["y"], 0.0)
        if last_pose is not None:
            state.pose.position.z = last_pose.position.z
        state.pose.orientation = orientation
        state.twist = Twist()
        state.reference_frame = "world"

        try:
            set_model_state(state)
            rospy.loginfo(
                "[%s] Repositioned to (%.2f, %.2f, %.2f rad)",
                self.robot_name,
                self.pose["x"],
                self.pose["y"],
                self.pose["yaw"],
            )
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to set robot pose: %s", exc)


def main():
    rospy.init_node("seeded_robot_initializer", log_level=rospy.INFO)
    rospy.loginfo("TIAGo seeded robot initializer module [OK]")
    initializer = SeededRobotInitializer()
    initializer.run()
