import numpy as np
import typing as T

from enum import Enum
from dataclasses import dataclass
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from scipy.interpolate import splev

from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl
from asl_tb3_lib.control import BaseController
from asl_tb3_lib.grids import snap_to_grid, StochOccupancyGrid2D
from asl_tb3_lib.math_utils import wrap_angle, distance_linear, distance_angular


@dataclass
class TrajectoryPlan:
    """ Data structure for holding a trajectory plan comming for A* planner and
        a trajectory smoother

    See https://docs.python.org/3.10/library/dataclasses.html for how to work
    with dataclasses. In short, __init__ function is implicitly created with
    arguments replaced by the following properties. For example, you can
    create a trajectory plan with

    ```
    my_plan = TrajectoryPlan(path=..., path_x_spline=..., path_y_spline=..., duration=...)
    ```
    """

    # raw planned path from A*
    path: np.ndarray

    # cubic spline fit of the x and y trajectory,
    # should be return values from scipy.interpolate.splrep
    #   see https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.splrep.html
    path_x_spline: T.Tuple[np.ndarray, np.ndarray, int]
    path_y_spline: T.Tuple[np.ndarray, np.ndarray, int]

    # time duration of the trajectory plan
    duration: float

    def desired_state(self, t: float) -> TurtleBotState:
        """ Get state from the plan at specified time point

        Args:
            t (float): time in [seconds]

        Returns:
            TurtleBotState: desired state at t
        """
        x_d = splev(t, self.path_x_spline, der=1)
        y_d = splev(t, self.path_y_spline, der=1)

        return TurtleBotState(
            x=float(splev(t, self.path_x_spline, der=0)),
            y=float(splev(t, self.path_y_spline, der=0)),
            theta=float(np.arctan2(y_d, x_d)),
        )

    def smoothed_path(self, dt: float = 0.1) -> np.ndarray:
        """ Get the full smoothed path sampled with fixed time steps

        Args:
            dt (float): sampling duration in [seconds]

        Returns:
            np.ndarray: smoothed trajectory sampled @ dt
        """
        ts = np.arange(0., self.duration, dt)
        path = np.zeros((ts.shape[0], 2))
        path[:, 0] = splev(ts, self.path_x_spline, der=0)
        path[:, 1] = splev(ts, self.path_y_spline, der=0)

        return path


class NavMode(Enum):
    """ Navigation Mode """

    IDLE = 0    # robot does not move
    ALIGN = 1   # control angular velocity only to align orientation with the planned initial state
    TRACK = 2   # track planned trajectory using a tracking controller
    PARK = 3    # use pose stabilization controller when close to the planned goal state


class BaseNavigator(BaseController):
    """ Student can inherit from this class to build a navigator node

    This node takes target pose from /cmd_nav, and control the robot towards the
    target pose using a switching controller that accounts for obstacles
    """

    def __init__(self, node_name: str = "navigator") -> None:
        super().__init__(node_name)

        self.mode = NavMode.IDLE

        self.is_planned = False
        self.plan_start_time = self.get_clock().now()
        self.plan: T.Optional[TrajectoryPlan] = None
        self.goal: T.Optional[TurtleBotState] = None
        self.occupancy: T.Optional[StochOccupancyGrid2D] = None

        self.cmd_nav_sub = self.create_subscription(TurtleBotState, "/cmd_nav", self.replan, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.planned_path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.smoothed_path_pub = self.create_publisher(Path, "/smoothed_path", 10)

        # parameters
        self.declare_parameter("theta_start_thresh", 0.05)  # threshold for heading controller
        self.declare_parameter("plan_thresh", 0.3)          # replan if at least this far from planned trajectory
        self.declare_parameter("near_thresh", 0.2)          # threshold to switch to pose stabilization controller
        self.declare_parameter("at_thresh", 0.02)           # maximum distance delta from goal
        self.declare_parameter("at_thresh_theta", 0.05)     # maximum angle delta from goal
        self.declare_parameter("plan_resolution", 0.1)      # resolution for A* planner in [m]
        self.declare_parameter("plan_horizon", 10.0)        # maximum grid dimension for planning

    def replan(self, goal: TurtleBotState) -> None:
        """ Re-plan the path towards some goal state

        Args:
            goal (TurtleBotState): goal state
        """
        if self.occupancy is None:
            self.get_logger().warn("Unable to replan: occupancy map not yet available")
            return

        # no need to plan if close to target
        self.goal = goal
        if self.near_goal():
            self.is_planned = True
            self.switch_mode(NavMode.PARK)
            return

        new_plan = self.compute_trajectory_plan(
            state=self.state,
            goal=goal,
            occupancy=self.occupancy,
            resolution=self.get_parameter("plan_resolution").value,
            horizon=self.get_parameter("plan_horizon").value,
        )
        if new_plan is None:
            self.is_planned = False
            self.get_logger().warn("Replanning failed")
            return

        self.is_planned = True
        self.plan = new_plan
        self.get_logger().info(f"Replanned to {goal}")

        self.publish_planned_path()
        self.publish_smooth_path()

        # no need to use heading controller if already aligned
        if self.aligned():
            self.plan_start_time = self.get_clock().now()
            self.switch_mode(NavMode.TRACK)
        else:
            self.switch_mode(NavMode.ALIGN)

    def publish_planned_path(self) -> None:
        """ Publish planned path from A* """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for state in self.plan.path:
            pose_st = PoseStamped()
            pose_st.header.frame_id = "map"
            pose_st.pose.position.x = state[0]
            pose_st.pose.position.y = state[1]
            pose_st.pose.orientation.w = 1.0
            path_msg.poses.append(pose_st)
        self.planned_path_pub.publish(path_msg)

    def publish_smooth_path(self) -> None:
        """ Publish smoothed trajectory """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for state in self.plan.smoothed_path():
            pose_st = PoseStamped()
            pose_st.header.frame_id = "map"
            pose_st.pose.position.x = state[0]
            pose_st.pose.position.y = state[1]
            pose_st.pose.orientation.w = 1.0
            path_msg.poses.append(pose_st)
        self.smoothed_path_pub.publish(path_msg)

    def map_callback(self, msg: OccupancyGrid) -> None:
        """ Callback triggered when the map is updated

        Args:
            msg (OccupancyGrid): updated map message
        """
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=7,
            probs=msg.data,
        )

        # replan if the new map updates causes collision in the original plan
        if self.is_planned and not all([self.occupancy.is_free(s) for s in self.plan.path]):
            self.is_planned = False
            self.replan(self.goal)

    def switch_mode(self, new_mode: NavMode):
        """ Switch to some navigation mode

        Args:
            new_mode (NavMode): new navigation mode
        """
        if self.mode != new_mode:
            self.get_logger().info(f"Switching mode: {self.mode} -> {new_mode}")
            self.mode = new_mode

    def near_goal(self) -> bool:
        """ Check if current state is near the goal state in linear distance

        Returns:
            bool: True if the linear distance between current state and goal state
                  is below some threshold, False otherwise
        """
        near_thresh = self.get_parameter("near_thresh").value
        return distance_linear(self.state, self.goal) < near_thresh

    def at_goal(self) -> bool:
        """ Check if the current state is at the goal state

        Returns:
            bool: True if the linear distance between current state and goal state
                  is below some threshold, False otherwise
        """
        at_thresh = self.get_parameter("at_thresh").value
        at_thresh_theta = self.get_parameter("at_thresh_theta").value
        return distance_linear(self.state, self.goal) < at_thresh and \
               distance_angular(self.state, self.goal) < at_thresh_theta

    def aligned(self) -> bool:
        """ Check if the current state is aligned to the initial planned state in orientation

        Returns:
            bool: True if the angular distance between current state and the planned
                  initial state is below some threshold
        """
        theta_start_thresh = self.get_parameter("theta_start_thresh").value
        return distance_angular(self.state, self.plan.desired_state(0.0)) < theta_start_thresh

    def close_to_plan(self) -> bool:
        """ Check whether the current state is staying close to the planned trajectory

        Returns:
            bool: True if the linear distance between current state and the planned
                  state at current time step is below some threshold, False otherwise
        """
        plan_thresh = self.get_parameter("plan_thresh").value
        t = (self.get_clock().now() - self.plan_start_time).nanoseconds * 1e-9
        return distance_linear(self.state, self.plan.desired_state(t)) < plan_thresh

    def compute_control(self) -> TurtleBotControl:
        """ High-level function for computing control targets.

        This function
            1) manipulates navigation mode transitions
            2) calls the corresponding controller depending on the current navigation mode

        Returns:
            TurtleBotControl: control target to send to the robot
        """
        # state machine switch
        if self.mode == NavMode.ALIGN:
            if self.aligned():
                self.plan_start_time = self.get_clock().now()
                self.switch_mode(NavMode.TRACK)
        elif self.mode == NavMode.TRACK:
            if self.near_goal():
                self.switch_mode(NavMode.PARK)
            elif self.get_clock().now() - self.plan_start_time > Duration(seconds=self.plan.duration):
                self.get_logger().info("Replanning because out of time or stuck")
                self.is_planned = False
                self.replan(self.goal)
            elif not self.close_to_plan():
                self.get_logger().info("Replanning because far from planned trajectory")
                self.is_planned = False
                self.replan(self.goal)
        elif self.mode == NavMode.PARK:
            if self.at_goal():
                self.is_planned = False
                self.switch_mode(NavMode.IDLE)

        # compute control
        if self.mode == NavMode.ALIGN:
            return self.compute_heading_control()
        elif self.mode == NavMode.TRACK:
            return self.compute_trajectory_tracking_control(
                state=self.state,
                plan=self.plan,
                t=(self.get_clock().now() - self.plan_start_time).nanoseconds * 1e-9,
            )
        elif self.mode == NavMode.PARK:
            return self.compute_pose_stabilize_control(self.state, self.goal)
        else:   # NavMode.IDLE:
            return TurtleBotControl()

    def can_compute_control(self) -> bool:
        """ Can compute for a control only when planning succeed upon receiving a goal state

        Returns:
            bool: True if planning succeed on a goal state, False otherwise
        """
        return self.is_planned

    def compute_heading_control(self) -> TurtleBotControl:
        """ Compute only orientation target (used for NavMode.ALIGN)

        Returns:
            TurtleBotControl: control target
        """
        kp = 2.0  # control gain for heading controller
        om_max = self.om_max

        err = wrap_angle(self.plan.desired_state(0.0).theta - self.state.theta)
        om = kp * err
        om = np.clip(om, -om_max, om_max)

        return TurtleBotControl(
            v=0.0,
            omega=om,
        )

    def compute_pose_stabilize_control(self,
        state: TurtleBotState,
        goal: TurtleBotState
    ) -> TurtleBotControl:
        """ Compute control target using a pose stabilization controller.

        HINT: you can copy your code from `compute_control_with_goal` in ealier sections

        Args:
            state (TurtleBotState): current robot state
            goal (TurtleBotState): current goal state

        Returns:
            TurtleBotControl: control command
        """
        raise NotImplementedError("You need to implement this!")

    def compute_trajectory_tracking_control(self,
        state: TurtleBotState,
        plan: TrajectoryPlan,
        t: float,
    ) -> TurtleBotControl:
        """ Compute control target using a trajectory tracking controller

        Args:
            state (TurtleBotState): current robot state
            plan (TrajectoryPlan): planned trajectory
            t (float): current timestep

        Returns:
            TurtleBotControl: control command
        """
        raise NotImplementedError("You need to implement this!")

    def compute_trajectory_plan(self,
        state: TurtleBotState,
        goal: TurtleBotState,
        occupancy: StochOccupancyGrid2D,
        resolution: float,
        horizon: float,
    ) -> T.Optional[TrajectoryPlan]:
        """ Compute a trajectory plan using A* and cubic spline fitting

        Args:
            state (TurtleBotState): state
            goal (TurtleBotState): goal
            occupancy (StochOccupancyGrid2D): occupancy
            resolution (float): resolution
            horizon (float): horizon

        Returns:
            T.Optional[TrajectoryPlan]:
        """
        raise NotImplementedError("You need to implement this!")

