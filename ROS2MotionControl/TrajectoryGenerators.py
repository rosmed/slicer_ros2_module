"""
TrajectoryGenerators.py
-----------------------
Base class and built-in trajectory generators for the ROS2 Motion Control module.

To add a custom generator, subclass TrajectoryGeneratorBase and call register() with an
instance of your class.  The Trajectory tab will automatically pick up all registered
generators and expose them in its drop-down menu.

Trajectory format – ``plan()`` returns a ``vtkMoveitMsgsRobotTrajectory`` (or ``None``).
The joint-trajectory data can be read via::

    jt = traj.GetJointTrajectory()
    joint_names = list(jt.GetJointNames())
    for pt in jt.GetPoints():
        positions = list(pt.GetPositions())
        ts = pt.GetTimeFromStart()          # vtkBuiltinInterfacesDuration
        t  = ts.GetSec() + ts.GetNanosec() * 1e-9

All position values are in radians; time is in seconds.
"""



# ---------------------------------------------------------------------------
# VTK trajectory builder
# ---------------------------------------------------------------------------

def _build_vtk_trajectory(joint_names, points_data):
    """Build a vtkMoveitMsgsRobotTrajectory from plain Python data.

    Parameters
    ----------
    joint_names : list[str]
    points_data : list[dict]  – each dict has at least "positions" (list[float])
                                and "time_from_start" (float, seconds); optionally
                                "velocities" and "accelerations".

    Returns
    -------
    vtkMoveitMsgsRobotTrajectory or None on import failure.
    """
    try:
        from vtkSlicerROS2ModuleMRMLPython import (
            vtkMoveitMsgsRobotTrajectory,
            vtkTrajectoryMsgsJointTrajectoryPoint,
            vtkBuiltinInterfacesDuration,
        )
    except ImportError as exc:
        print(f"_build_vtk_trajectory: cannot import vtk types – {exc}")
        return None

    traj = vtkMoveitMsgsRobotTrajectory()
    jt = traj.GetJointTrajectory()
    jt.SetJointNames(list(joint_names))

    vtk_points = []
    for pd in points_data:
        pt = vtkTrajectoryMsgsJointTrajectoryPoint()
        pt.SetPositions(pd.get("positions", []))
        pt.SetVelocities(pd.get("velocities", []))
        pt.SetAccelerations(pd.get("accelerations", []))
        t = pd.get("time_from_start", 0.0)
        dur = vtkBuiltinInterfacesDuration()
        dur.SetSec(int(t))
        dur.SetNanosec(int(round((t - int(t)) * 1e9)))
        pt.SetTimeFromStart(dur)
        vtk_points.append(pt)

    jt.SetPoints(vtk_points)
    return traj


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

            def plan(self, joint_names, start_positions, goal_positions, **kwargs):
                ...
                # return vtkMoveitMsgsRobotTrajectory or None

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
        **kwargs,
    ) -> "vtkMoveitMsgsRobotTrajectory | None":
        """Generate a joint-space trajectory.

        Parameters
        ----------
        joint_names:
            Ordered list of joint name strings.
        start_positions:
            Current joint positions (radians), same length as *joint_names*.
        goal_positions:
            Goal joint positions (radians), same length as *joint_names*.
        **kwargs:
            Generator-specific extras.  Built-in keys forwarded by the module:
              ``robot``               – vtkMRMLROS2RobotNode (provides joint limits)
              ``motion_control_node`` – vtkMRMLROS2MotionControlNode
              ``plan_group``          – MoveIt planning group name (str)
              ``planning_time``       – MoveIt planning time in seconds (float)
              ``velocity_scaling``    – MoveIt velocity scaling factor (float)
              ``acceleration_scaling``– MoveIt acceleration scaling factor (float)

        Returns
        -------
        vtkMoveitMsgsRobotTrajectory or None
            Trajectory object, or *None* on failure.
        """
        raise NotImplementedError


# ---------------------------------------------------------------------------
# MoveIt generator
# ---------------------------------------------------------------------------

class MoveItTrajectoryGenerator(TrajectoryGeneratorBase):
    """Trajectory generator that delegates to the MoveIt move_group action.

    Requires that a ``/move_group`` ROS 2 node is running and that the robot
    node was set up with ``SetupIKMoveIt()``.
    """

    @property
    def name(self) -> str:
        return "MoveIt"

    def plan(self, joint_names, start_positions, goal_positions, **kwargs):
        motion_control_node = kwargs.get("motion_control_node")
        plan_group = kwargs.get("plan_group", "")
        planning_time = float(kwargs.get("planning_time", 5.0))
        vel_scaling = float(kwargs.get("velocity_scaling", 0.5))
        acc_scaling = float(kwargs.get("acceleration_scaling", 0.5))

        if motion_control_node is None:
            print("MoveItTrajectoryGenerator: no motion control node provided")
            return None

        if not plan_group:
            print("MoveItTrajectoryGenerator: planning group name is empty")

       # traj = motion_control_node.PlanMoveItTrajectory(plan_group, goal_positions,
        #                                                vel_scaling, acc_scaling, planning_time)
        # MS - TESTING: trying to fix micromate
        traj = motion_control_node.PlanMoveItTrajectory(plan_group, joint_names, goal_positions,
                                                vel_scaling, acc_scaling, planning_time)
                                                
        if traj is None:
            return None

        # Return the vtk object directly — no intermediate conversion needed
        jt = traj.GetJointTrajectory()
        if not jt.GetPoints():
            return None
        return traj


# ---------------------------------------------------------------------------
# Simple generator
# ---------------------------------------------------------------------------

class SimpleTrajectoryGenerator(TrajectoryGeneratorBase):
    """Simple joint-space trajectory generator using a bang-bang velocity profile.

    Each joint accelerates to its maximum velocity (from the robot node's URDF
    limits), cruises, then decelerates symmetrically.  The total motion time is
    set by the joint that requires the longest time, so all joints arrive at the
    goal simultaneously.

    Joint position and velocity limits are read directly from the
    ``vtkMRMLROS2RobotNode`` passed as ``robot=`` in the ``plan()`` call — no
    URDF re-parsing in Python.  Falls back to *default_max_velocity* if no
    robot node is supplied.

    Parameters
    ----------
    num_waypoints : int
        Number of evenly-spaced time samples in the output trajectory (default 50).
    default_max_velocity : float
        Fallback velocity in rad/s when no robot node is provided (default 1.0).
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

    # URDF parsing has been removed from this class.
    # Limit dicts are pre-computed by the widget and passed as kwargs to plan().

    @staticmethod
    def _bb_alpha(tau: float) -> float:
        """Normalised position (0→1) for bang-bang profile at tau∈[0,1]."""
        if tau <= 0.5:
            return 2.0 * tau * tau
        else:
            t2 = 1.0 - tau
            return 1.0 - 2.0 * t2 * t2

    @staticmethod
    def _bb_dalpha(tau: float) -> float:
        """dα/dτ: normalised velocity coefficient. Multiply by Δ/T for actual velocity."""
        if tau <= 0.5:
            return 4.0 * tau
        else:
            return 4.0 * (1.0 - tau)

    @staticmethod
    def _bb_d2alpha(tau: float) -> float:
        """d²α/dτ²: normalised acceleration coefficient. Multiply by Δ/T² for actual acceleration."""
        return 4.0 if tau <= 0.5 else -4.0

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def plan(self, joint_names, start_positions, goal_positions, **kwargs):
        import math as _math
        n = len(joint_names)
        if len(start_positions) != n or len(goal_positions) != n:
            print("SimpleTrajectoryGenerator: joint count mismatch")
            return None

        # Retrieve limits directly from the robot node (already parsed by C++).
        # Falls back to empty dicts if no robot node is provided.
        robot = kwargs.get("robot")
        if robot is not None:
            # MS: TESTING
            # lower_list = list(robot.GetJointLowerPositionLimits())
            # upper_list = list(robot.GetJointUpperPositionLimits())
            # vel_list   = list(robot.GetJointVelocityLimits())
            lower_list = list(robot.GetAllControllableJointLowerLimits())
            upper_list = list(robot.GetAllControllableJointUpperLimits())
            vel_list   = list(robot.GetAllControllableJointVelocityLimits())
            # END - MS: TESTING
            pos_limits = {name: (lower_list[i], upper_list[i]) for i, name in enumerate(joint_names)}
            vel_limits = {name: v for name, v in zip(joint_names, vel_list) if v > 0.0}
        else:
            pos_limits = {}
            vel_limits = {}

        # For each joint choose the shortest angular path that stays within position
        # limits (if known). If neither direction is within limits, clamp the goal.
        deltas = []
        for i, name in enumerate(joint_names):
            s = start_positions[i]
            g = goal_positions[i]
            raw_delta = g - s

            # Shortest angular delta (wrap to [-π, π])
            short = (raw_delta + _math.pi) % (2 * _math.pi) - _math.pi
            # Long way around
            long_ = short - 2 * _math.pi * (1 if short > 0 else -1)

            if name in pos_limits:
                lower, upper = pos_limits[name]
                short_goal = s + short
                long_goal  = s + long_

                short_ok = lower - 1e-9 <= short_goal <= upper + 1e-9
                long_ok  = lower - 1e-9 <= long_goal  <= upper + 1e-9

                if short_ok:
                    chosen = short
                elif long_ok:
                    chosen = long_
                else:
                    # Neither path ends within limits — clamp to nearest limit.
                    clamped_goal = max(lower, min(upper, g))
                    chosen = clamped_goal - s
                    print(f"SimpleTrajectoryGenerator: joint '{name}' goal {g:.3f} rad "
                          f"clamped to [{lower:.3f}, {upper:.3f}] → {clamped_goal:.3f} rad")
            else:
                # No position limit info — just take shortest path.
                chosen = short

            deltas.append(chosen)

        # Total motion time is determined by the slowest joint.
        # For a bang-bang profile: T = 2 * |Δθ| / v_max.
        total_time = 0.0
        for i, name in enumerate(joint_names):
            delta = abs(deltas[i])
            max_vel = vel_limits.get(name, self.default_max_velocity)
            joint_time = (2.0 * delta / max_vel) if delta > 1e-9 else 0.0
            if joint_time > total_time:
                total_time = joint_time

        # Already at goal – return a single-point trajectory.
        if total_time < 1e-9:
            return _build_vtk_trajectory(
                joint_names,
                [{"positions": list(goal_positions), "velocities": [0.0] * n, "time_from_start": 0.0}],
            )

        # Sample the bang-bang profile at num_waypoints uniform time instants.
        # Velocities and accelerations are derived analytically so that MoveIt's
        # joint_trajectory_controller accepts the trajectory without aborting.
        points_data = []
        for k in range(self.num_waypoints):
            tau = k / (self.num_waypoints - 1)
            alpha    = self._bb_alpha(tau)
            d_alpha  = self._bb_dalpha(tau)
            d2_alpha = self._bb_d2alpha(tau)
            positions     = [start_positions[i] + alpha * deltas[i] for i in range(n)]
            velocities    = [(deltas[i] / total_time) * d_alpha  for i in range(n)]
            accelerations = [(deltas[i] / (total_time ** 2)) * d2_alpha for i in range(n)]
            points_data.append({
                "positions":     positions,
                "velocities":    velocities,
                "accelerations": accelerations,
                "time_from_start": tau * total_time,
            })

        return _build_vtk_trajectory(joint_names, points_data)


# ---------------------------------------------------------------------------
# Register built-in generators (order determines drop-down order)
# ---------------------------------------------------------------------------

register(MoveItTrajectoryGenerator())
register(SimpleTrajectoryGenerator())
