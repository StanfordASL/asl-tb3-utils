import math

from geometry_msgs.msg import Quaternion, Transform, Pose
from asl_tb3_msgs.msg import TurtleBotState


def quaternion_to_yaw(q: Quaternion) -> float:
    return math.atan2(2. * q.w * q.z, q.w ** 2 - q.z ** 2)

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0
    q.y = 0
    q.z = math.sin(yaw / 2)
    q.w = math.cos(yaw / 2)

    return q

def transform_to_state(t: Transform) -> TurtleBotState:
    state = TurtleBotState()
    state.x = t.translation.x
    state.y = t.translation.y
    state.theta = quaternion_to_yaw(t.rotation)

    return state

def state_to_transform(state: TurtleBotState) -> Transform:
    t = Transform()
    t.translation.x = state.x
    t.translation.y = state.y
    t.translation.z = 0
    t.rotation = yaw_to_quaternion(state.theta)

    return t

def pose_to_state(p: Pose) -> TurtleBotState:
    state = TurtleBotState()
    state.x = p.position.x
    state.y = p.position.y
    state.theta = quaternion_to_yaw(p.orientation)

    return state

def state_to_pose(state: TurtleBotState) -> Pose:
    p = Pose()
    p.position.x = state.x
    p.position.y = state.y
    p.position.z = 0
    p.orientation = yaw_to_quaternion(state.theta)

    return p
