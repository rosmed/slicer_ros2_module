"""
TrajectoryGenerators.py
-----------------------
Base class and built-in trajectory generators for the ROS2 Motion Control module.

To add a custom generator, subclass TrajectoryGeneratorBase and call register() with an
instance of your class.  The Trajectory tab will automatically pick up all registered
generators and expose them in its drop-down menu.

Trajectory format (JSON string returned by plan()):

    {
        "joint_names": ["joint1", "joint2", ...],
        "points": [
            {"positions": [<float>, ...], "time_from_start": <float>},
            ...
        ]
    }

All position values are in radians; time_from_start is in seconds.
"""

import json
import math
import xml.etree.ElementTree as ET


# ---------------------------------------------------------------------------
# Public registry
# ---------------------------------------------------------------------------

_REGISTRY: list = []


def register(generator_instance: "TrajectoryGeneratorBase") -> None:
    """Register a TrajectoryGeneratorBase instance.

    Call this at module level (or at import time) from your own script so the
    Trajectory tab picks up the generator automatically::

        from TrajectoryGenerators import register, TrajectoryGeneratorBase

        class MyGenerator(TrajectoryGeneratorBase):
            @property
            def name(self):
                return "My Generator"

            def plan(self, joint_names, start_positions, goal_positions, urdf_xml="", **kwargs):
                ...  # return JSON string or None

        register(MyGenerator())
    """
    _REGISTRY.append(generator_instance)


def get_all() -> list:
    """Return a copy of the current generator registry."""
    return list(_REGISTRY)


# ---------------------------------------------------------------------------
# Base class
# ---------------------------------------------------------------------------

class TrajectoryGeneratorBase:
    """Abstract base class for joint-space trajectory generators.

    Subclass this and implement *name* and *plan()*, then call
    ``register(MyGenerator())`` so the Trajectory tab discovers it.
    """

    @property
    def name(self) -> str:
        """Human-readable name shown in the Generator drop-down."""
        raise NotImplementedError

    def plan(
        self,
        joint_names: list,
        start_positions: list,
        goal_positions: list,
        urdf_xml: str = "",
        **kwargs,
    ) -> "str | None":
        """Generate a joint-space trajectory.

        Parameters
        ----------
        joint_names:
            Ordered list of joint name strings.
        start_positions:
            Current joint positions (radians), same length as *joint_names*.
        goal_positions:
            Goal joint positions (radians), same length as *joint_names*.
        urdf_xml:
            Full URDF XML string for the robot (may be empty).
        **kwargs:
            Generator-specific extras.  Built-in keys forwarded by the module:
              ``robot``       – vtkMRMLROS2RobotNode
              ``plan_group``  – MoveIt planning group name (str)

        Returns
        -------
        str or None
            JSON string conforming to the trajectory format described at the
            top of this file, or *None* on failure.
        """
        raise NotImplementedError


# ---------------------------------------------------------------------------
# MoveIt generator
# ---------------------------------------------------------------------------

class MoveItTrajectoryGenerator(TrajectoryGeneratorBase):
    """Trajectory generator that delegates to the MoveIt move_group action.

    Requires that a ``/move_group`` ROS 2 node is running and that the robot
    node was set up with ``setupIKmoveit()``.
    """

    @property
    def name(self) -> str:
        return "MoveIt"

    def plan(self, joint_names, start_positions, goal_positions, urdf_xml="", **kwargs):
        robot = kwargs.get("robot")
        plan_group = kwargs.get("plan_group", "")

        if robot is None:
            print("MoveItTrajectoryGenerator: no robot node provided")
            return None

        if not plan_group:
            print("MoveItTrajectoryGenerator: planning group name is empty")

        return robot.PlanMoveItTrajectoryJSON(plan_group, goal_positions)


# ---------------------------------------------------------------------------
# Simple generator
# ---------------------------------------------------------------------------

class SimpleTrajectoryGenerator(TrajectoryGeneratorBase):
    """Simple joint-space trajectory generator using a bang-bang velocity profile.

    Each joint accelerates to its maximum velocity (read from the URDF ``<limit
    velocity="..."/>`` attribute), cruises, then decelerates symmetrically.
    The total motion time is set by the joint that requires the longest time,
    so all joints arrive at the goal simultaneously.

    When no URDF velocity limit is found for a joint, *DEFAULT_MAX_VELOCITY* is
    used as a fallback.

    Parameters
    ----------
    num_waypoints : int
        Number of evenly-spaced time samples in the output trajectory (default 50).
    default_max_velocity : float
        Fallback velocity in rad/s when the URDF has no limit for a joint
        (default 1.0).
    """

    def __init__(self, num_waypoints: int = 50, default_max_velocity: float = 1.0):
        self.num_waypoints = num_waypoints
        self.default_max_velocity = default_max_velocity

    @property
    def name(self) -> str:
        return "Simple joint trajectory"

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _parse_velocity_limits(self, urdf_xml: str) -> dict:
        """Return {joint_name: max_velocity_rad_s} from URDF."""
        limits = {}
        if not urdf_xml:
            return limits
        try:
            root = ET.fromstring(urdf_xml)
            for joint in root.findall("joint"):
                jtype = joint.get("type", "")
                name = joint.get("name", "")
                if jtype == "fixed" or not name:
                    continue
                limit_elem = joint.find("limit")
                if limit_elem is None:
                    continue
                vel = limit_elem.get("velocity")
                if vel is not None:
                    v = float(vel)
                    if v > 0:
                        limits[name] = v
        except Exception as exc:
            print(f"SimpleTrajectoryGenerator: failed to parse URDF velocity limits: {exc}")
        return limits

    @staticmethod
    def _bb_alpha(tau: float) -> float:
        """Normalised position (0 → 1) for a bang-bang profile at normalised time tau ∈ [0, 1].

        Phase 1 (0 ≤ τ ≤ 0.5): constant positive acceleration → quadratic rise.
        Phase 2 (0.5 < τ ≤ 1): constant negative acceleration → quadratic fall.
        """
        if tau <= 0.5:
            return 2.0 * tau * tau
        else:
            t2 = 1.0 - tau
            return 1.0 - 2.0 * t2 * t2

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def plan(self, joint_names, start_positions, goal_positions, urdf_xml="", **kwargs):
        n = len(joint_names)
        if len(start_positions) != n or len(goal_positions) != n:
            print("SimpleTrajectoryGenerator: joint count mismatch")
            return None

        vel_limits = self._parse_velocity_limits(urdf_xml)

        # Total motion time is determined by the slowest joint.
        # For a bang-bang profile the peak velocity equals the URDF limit and the
        # distance travelled is  Δθ = 0.5 * v_max * T  =>  T = 2 * Δθ / v_max.
        total_time = 0.0
        for i, name in enumerate(joint_names):
            delta = abs(goal_positions[i] - start_positions[i])
            max_vel = vel_limits.get(name, self.default_max_velocity)
            joint_time = (2.0 * delta / max_vel) if delta > 1e-9 else 0.0
            if joint_time > total_time:
                total_time = joint_time

        # Already at goal – return a single-point trajectory.
        if total_time < 1e-9:
            point = {
                "positions": list(goal_positions),
                "velocities": [0.0] * n,
                "time_from_start": 0.0,
            }
            return json.dumps({"joint_names": list(joint_names), "points": [point]})

        # Sample the bang-bang profile at num_waypoints uniform time instants.
        points = []
        for k in range(self.num_waypoints):
            tau = k / (self.num_waypoints - 1)
            alpha = self._bb_alpha(tau)
            positions = [
                start_positions[i] + alpha * (goal_positions[i] - start_positions[i])
                for i in range(n)
            ]
            points.append({"positions": positions, "time_from_start": tau * total_time})

        return json.dumps({"joint_names": list(joint_names), "points": points})


# ---------------------------------------------------------------------------
# Register built-in generators (order determines drop-down order)
# ---------------------------------------------------------------------------

register(MoveItTrajectoryGenerator())
register(SimpleTrajectoryGenerator())
