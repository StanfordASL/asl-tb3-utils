import numpy as np

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl
from asl_tb3_lib.tf_utils import transform_to_state


class BaseController(Node):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.state_ready = False
        self.state = TurtleBotState()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.control_timer = self.create_timer(0.1, self.publish_control)  # 10 Hz control loop

        self.declare_parameter("v_max", 0.2)    # maximum linear velocity
        self.declare_parameter("om_max", 0.4)   # maximum angular velocity

    @property
    def v_max(self) -> float:
        return self.get_parameter("v_max").value

    @property
    def om_max(self) -> float:
        return self.get_parameter("om_max").value

    def try_get_latest_pose(self) -> None:
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
            self.state = transform_to_state(t.transform)
            self.state_ready = True
        except TransformException as e:
            pass

    def publish_control(self) -> None:
        self.try_get_latest_pose()
        if not self.state_ready:
            self.get_logger().debug("Latest pose not yet ready")
            return

        if not self.can_compute_control():
            self.get_logger().debug("Cannot compute control: publishing zero control")
            self.cmd_vel_pub.publish(Twist())
            return

        control = self.compute_control()

        v_max = self.v_max
        om_max = self.om_max

        twist = Twist()
        twist.linear.x = np.clip(control.v, -v_max, v_max)
        twist.angular.z = np.clip(control.omega, -om_max, om_max)
        self.cmd_vel_pub.publish(twist)

    def can_compute_control(self) -> bool:
        return True

    def compute_control(self) -> TurtleBotControl:
        raise NotImplementedError("Calling abstract function")


class BasePoseStabilizer(BaseController):
    def __init__(self, node_name: str = "pose_stabilizer") -> None:
        super().__init__(node_name)

        self.goal = TurtleBotState()
        self.goal_set = False

        self.cmd_pose_sub = self.create_subscription(
            TurtleBotState, "/cmd_pose", self.cmd_pose_callback, 10)

    def cmd_pose_callback(self, msg: TurtleBotState) -> None:
        if not self.goal_set or self.goal != msg:
            self.goal = msg
            self.goal_set = True
            self.get_logger().info(f"New command pose received: {msg}")

    def compute_control(self) -> TurtleBotControl:
        return self.compute_control_with_goal(self.state, self.goal)

    def can_compute_control(self) -> bool:
        return self.goal_set

    def compute_control_with_goal(self,
        state: TurtleBotState,
        goal: TurtleBotState
    ) -> TurtleBotControl:
        raise NotImplementedError("You need to implement this!")


