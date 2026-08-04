import cProfile
import math
import random
from typing import Dict, List, Optional

import numpy as np
import rospy
import crowd_navigation_msgs.msg
from shapely.geometry import MultiPolygon, Point as ShapelyPoint, Polygon

from crowd_navigation_core.Hparams import Hparams


class TargetSampler:
    """Deterministically samples goal poses far from the robot's initial pose."""

    def __init__(self, hparams: Hparams):
        self.hparams = hparams
        self.pub = rospy.Publisher(
            "/target_position",
            crowd_navigation_msgs.msg.TargetPosition,
            queue_size=1000,
            latch=True,
        )
        rospy.Subscriber(
            "/target_achieved",
            crowd_navigation_msgs.msg.TargetAchieved,
            self.target_achieved_callback,
        )

        self.min_goal_distance = float(rospy.get_param("~min_goal_distance", 8.0))
        self.goal_clearance = float(rospy.get_param("~goal_clearance", 0.4))
        self.goal_sampling_attempts = int(rospy.get_param("~goal_sampling_attempts", 200))
        self.initial_pose_timeout = float(rospy.get_param("~initial_pose_timeout", 30.0))
        self.one_shot = bool(rospy.get_param("~one_shot", True))

        self.initial_pose = self._wait_for_initial_pose()
        self.goal_area_indexes = self._resolve_goal_area_indexes()
        self._polygons = self._build_polygons(self.goal_area_indexes)

        base_seed = int(
            rospy.get_param(
                "~experiment_seed",
                rospy.get_param("/crowd_navigation_core/experiment_seed", 0),
            )
        )
        seed_offset = int(rospy.get_param("~goal_seed_offset", 100))
        self.rng = random.Random(base_seed + seed_offset)

        self.current_target: Optional[Dict[str, float]] = None

    def publish_next_target(self) -> None:
        if self.current_target is None:
            self.current_target = self._sample_goal()
            if self.current_target is None:
                rospy.logerr_throttle(5.0, "Unable to sample a valid goal; retrying")
                return
            rospy.loginfo(
                "Selected target (%.2f, %.2f) in area %d [distance %.2fm]",
                self.current_target["x"],
                self.current_target["y"],
                self.current_target["area_index"],
                self._distance_to_robot(self.current_target["x"], self.current_target["y"]),
            )
            self._publish_current_target()
            return

        if not self.one_shot:
            self._publish_current_target()

    def target_achieved_callback(self, msg: crowd_navigation_msgs.msg.TargetAchieved) -> None:
        if not msg.success:
            return
        if self.current_target is None:
            rospy.logwarn("Received target_achieved but no active goal is set")
            return
        rospy.loginfo(
            "Target (%.2f, %.2f) achieved, sampling a new one",
            self.current_target["x"],
            self.current_target["y"],
        )
        if self.one_shot:
            rospy.loginfo("One-shot mode enabled; no further targets will be generated")
            return
        self.current_target = None
        self.publish_next_target()

    def run(self) -> None:
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publish_next_target()
            rate.sleep()

    def _publish_current_target(self) -> None:
        if self.current_target is None:
            return
        target_msg = crowd_navigation_msgs.msg.TargetPosition(
            self.current_target["x"],
            self.current_target["y"],
        )
        self.pub.publish(target_msg)

    def _wait_for_initial_pose(self) -> Dict[str, float]:
        param_name = "/crowd_navigation_core/initial_pose"
        start_time = rospy.Time.now()
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            if rospy.has_param(param_name):
                pose = rospy.get_param(param_name)
                if {"x", "y"}.issubset(pose):
                    return {
                        "x": float(pose["x"]),
                        "y": float(pose["y"]),
                        "yaw": float(pose.get("yaw", 0.0)),
                    }
                rospy.logwarn_once(
                    "Initial pose param is missing required keys; waiting for update"
                )
            if (rospy.Time.now() - start_time).to_sec() > self.initial_pose_timeout:
                raise rospy.ROSException(
                    f"Timed out after {self.initial_pose_timeout:.1f}s waiting for initial pose"
                )
            rate.sleep()
        raise rospy.ROSException("Shutdown while waiting for initial pose")

    def _resolve_goal_area_indexes(self) -> List[int]:
        param_key = "~goal_area_indexes"
        if rospy.has_param(param_key):
            indices = rospy.get_param(param_key)
            if isinstance(indices, (list, tuple)):
                return [int(idx) for idx in indices]
            return [int(indices)]
        return list(range(len(getattr(self.hparams, "areas", []))))

    def _build_polygons(self, area_indexes: List[int]) -> Dict[int, Polygon]:
        polygons: Dict[int, Polygon] = {}
        areas = getattr(self.hparams, "areas", [])
        if not areas:
            raise rospy.ROSException("Hparams.areas is empty; cannot build goal regions")
        for idx in area_indexes:
            try:
                area = np.asarray(areas[idx])
            except IndexError as exc:
                raise rospy.ROSException(
                    f"Goal area index {idx} is invalid for current Hparams"
                ) from exc
            polygon = Polygon(area.tolist())
            shrunk = self._shrink_polygon(polygon)
            if shrunk.is_empty:
                rospy.logwarn(
                    "Goal clearance %.2fm removed area %d; reusing original polygon",
                    self.goal_clearance,
                    idx,
                )
                shrunk = polygon
            polygons[idx] = shrunk
        return polygons

    def _shrink_polygon(self, polygon: Polygon) -> Polygon:
        if self.goal_clearance <= 0.0:
            return polygon
        buffered = polygon.buffer(-self.goal_clearance)
        if buffered.is_empty:
            return Polygon()
        if isinstance(buffered, Polygon):
            return buffered
        if isinstance(buffered, MultiPolygon):
            largest = max(buffered.geoms, key=lambda geom: geom.area, default=None)
            return largest if largest is not None else Polygon()
        if hasattr(buffered, "geoms"):
            polys = [geom for geom in buffered.geoms if isinstance(geom, Polygon)]
            if polys:
                return max(polys, key=lambda geom: geom.area)
        return Polygon()

    def _sample_goal(self) -> Optional[Dict[str, float]]:
        for _ in range(self.goal_sampling_attempts):
            area_idx = self.rng.choice(self.goal_area_indexes)
            polygon = self._polygons.get(area_idx)
            if polygon is None or polygon.is_empty:
                continue
            x, y = self._sample_point_in_polygon(polygon)
            if self._is_far_enough(x, y):
                return {"x": x, "y": y, "area_index": area_idx}
        rospy.logwarn(
            "Failed to sample a goal %.2fm away after %d attempts; using fallback",
            self.min_goal_distance,
            self.goal_sampling_attempts,
        )
        return self._fallback_goal()

    def _fallback_goal(self) -> Optional[Dict[str, float]]:
        best_goal = None
        best_distance = -math.inf
        for area_idx, polygon in self._polygons.items():
            if polygon.is_empty:
                continue
            centroid = polygon.centroid
            distance = self._distance_to_robot(centroid.x, centroid.y)
            if distance > best_distance:
                best_goal = {"x": centroid.x, "y": centroid.y, "area_index": area_idx}
                best_distance = distance
        if best_goal is None:
            rospy.logerr("No valid polygons available for fallback goal")
            return None
        if best_distance < self.min_goal_distance:
            rospy.logwarn(
                "Fallback goal is only %.2fm from the robot (threshold %.2fm)",
                best_distance,
                self.min_goal_distance,
            )
        return best_goal

    def _is_far_enough(self, x: float, y: float) -> bool:
        return self._distance_to_robot(x, y) >= self.min_goal_distance

    def _distance_to_robot(self, x: float, y: float) -> float:
        dx = x - self.initial_pose["x"]
        dy = y - self.initial_pose["y"]
        return math.hypot(dx, dy)

    def _sample_point_in_polygon(self, polygon: Polygon) -> List[float]:
        min_x, min_y, max_x, max_y = polygon.bounds
        for _ in range(1000):
            x = self.rng.uniform(min_x, max_x)
            y = self.rng.uniform(min_y, max_y)
            if polygon.contains(ShapelyPoint(x, y)):
                return [x, y]
        centroid = polygon.centroid
        rospy.logwarn("Fell back to polygon centroid while sampling goal")
        return [centroid.x, centroid.y]


def main() -> None:
    rospy.init_node("tiago_target_sampler", log_level=rospy.INFO)
    rospy.loginfo("TIAGo target sampler module [OK]")
    sampler = TargetSampler(Hparams())
    prof_filename = "/tmp/target_sampler.prof"
    cProfile.runctx(
        "sampler.run()",
        globals=globals(),
        locals=locals(),
        filename=prof_filename,
    )
