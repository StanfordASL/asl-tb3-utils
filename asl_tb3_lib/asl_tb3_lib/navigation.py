import numpy as np
import typing as T

from enum import Enum
from dataclasses import dataclass
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from scipy.interpolate import splev

from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl
from .control import BaseController
from .grids import snap_to_grid, StochOccupancyGrid2D
from .math_utils import distance_linear, distance_angular


@dataclass(frozen=True)
class TrajectoryPlan:
    path: np.ndarray
    path_x_spline: T.Tuple[np.ndarray, np.ndarray, int]
    path_y_spline: T.Tuple[np.ndarray, np.ndarray, int]
    duration: float

    @property
    def init(self) -> TurtleBotState:
        x_d = splev(0., self.path_x_spline, der=1)
        y_d = splev(0., self.path_y_spline, der=1)

        return TurtleBotState(
            x=splev(0., self.path_x_spline, der=0),
            y=splev(0., self.path_y_spline, der=0),
            theta=np.arctan2(y_d, x_d),
        )


class NavMode(Enum):
    IDLE = 0
    ALIGN = 1
    TRACK = 2
    PARK = 3


class BaseNavigator(BaseController):

    def __init__(self, node_name: str = "navigator") -> None:
        super().__init__(node_name)

        self.mode = NavMode.IDLE

        self.is_planned = False
        self.plan_start_time = self.get_clock().now()
        self.plan: T.Optional[TrajectoryPlan] = None
        self.goal: T.Optional[TurtleBotState] = None
        self.occupancy: T.Optional[StochOccupancyGrid2D] = None

        self.cmd_nav_sub = self.create_subscription(TurtleBotState, "/cmd_nav", self.replan, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback)
        self.planned_path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.smoothed_path_pub = self.create_publisher(Path, "/smoothed_path", 10)

        # dynamic parameters
        self.declare_parameter("v_des", 0.12)   # desired cruising velocity
        self.declare_parameter("theta_start_thresh", 0.05)  # threshold for heading controller
        self.declare_parameter("start_pos_thresh", 0.2)     # replan if at least this far from planned starting position
        self.declare_parameter("near_thresh", 0.2)          # threshold to switch to pose stabilization controller
        self.declare_parameter("at_thresh", 0.02)           # maximum distance delta from goal
        self.declare_parameter("at_thresh_theta", 0.05)     # maximum angle delta from goal

        # static parameters
        self.plan_resolution = self.declare_parameter("plan_resolution", 0.1).value

    def replan(self, goal: TurtleBotState) -> None:
        # no need to replan if the new goal is the same
        if self.is_planned and self.goal == msg:
            return

        if self.occupancy is None:
            self.get_logger().warn("Unable to replan: occupancy map not yet available")
            return

        self.goal = goal
        self.is_planned = True
        self.get_logger().info(f"Replanned to {goal}")

        # no need to plan if the goal is close
        if self.near_goal():
            self.switch_mode(NavMode.PARK)
            return

        self.plan = self.compute_trajectory_plan(self.state, goal)
        #TODO publish path for visulization

        if self.aligned():
            self.switch_mode(NavMode.TRACK)
        else:
            self.switch_mode(NavMode.ALIGN)

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=7,
            probs=msg.data,
        )

        # replan if necessary
        if self.is_planned:
            # TODO
            # self.is_planned = False
            # self.replan(self.goal)
            pass

    def switch_mode(self, new_mode: NavMode):
        self.get_logger().info(f"Switching mode: {self.mode} -> {new_mode}")
        self.mode = new_mode

    def near_goal(self) -> bool:
        near_thresh = self.get_parameter("near_thresh").value
        return distance_linear(self.state, self.goal) < near_thresh

    def at_goal(self) -> bool:
        at_thresh = self.get_parameter("at_thresh").value
        at_thresh_theta = self.get_parameter("at_thresh_theta").value
        return distance_linear(self.state, self.goal) < at_thresh and \
               distance_angular(self.state, self.goal) < at_thresh_theta

    def aligned(self) -> bool:
        theta_start_thresh = self.get_parameter("theta_start_thresh").value
        return distance_angular(self.state, self.plan.init) < theta_start_thresh

    def close_to_plan_start(self) -> bool:
        start_pos_thresh = self.get_parameter("start_pos_thresh").value
        return distance_linear(self.state, self.plan.init) < start_pos_thresh

    def compute_control(self) -> TurtleBotControl:
        # state machine switch
        if self.mode == NavMode.ALIGN:
            if self.aligned():
                self.plan_start_time = self.get_clock().now()
                self.switch_mode(NavMode.TRACK)
        elif self.mode == NavMode.TRACK:
            if self.near_goal():
                self.switch_mode(NavMode.PARK)
            elif not self.close_to_plan_start():
                self.get_logger().info("Replanning because far from start")
                self.is_planned = False
                self.replan(self.goal)
            elif self.get_clock().now() - self.plan_start_time > Duration(self.plan.duration):
                self.get_logger().info("Replanning because out of time or stuck")
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
                t=(self.get_clock().now() - self.plan_start_time).nanoseconds * 1e-9,
            )
        elif self.mode == NavMode.PARK:
            return self.compute_pose_stabilize_control(self.state, self.goal)

        # should not get here
        self.get_logger().error(f"Cannot compute control in {self.mode}")
        return TurtleBotControl()

    def can_compute_control(self) -> bool:
        return self.is_planned and self.mode != NavMode.IDLE

    def compute_heading_control(self) -> TurtleBotControl:
        kp = 2.0  # control gain for heading controller
        om_max = self.om_max

        om = kp * err
        om = np.clip(om, -om_max, om_max)

        return TurtleBotControl(
            v=0,
            omega=om,
        )

    def compute_pose_stabilize_control(self,
        state: TurtleBotState,
        goal: TurtleBotState
    ) -> TurtleBotControl:
        raise NotImplementedError("You need to implement this!")

    def compute_trajectory_tracking_control(self,
        state: TurtleBotState,
        t: float
    ) -> TurtleBotControl:
        raise NotImplementedError("You need to implement this!")

    def compute_trajectory_plan(self,
        state: TurtleBotState,
        goal: TurtleBotState,
    ) -> TrajectoryPlan:
        raise NotImplementedError("You need to implement this!")

