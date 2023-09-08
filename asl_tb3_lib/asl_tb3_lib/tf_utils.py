import math

from geometry_msgs.msg import Quaternion, Transform, Pose
from asl_tb3_msgs.msg import TurtleBotState


def quaternion_to_yaw(q: Quaternion) -> float:
    """ Extract yaw rotation from a quaternion

    Args:
        q (Quaternion): q quaternion to convert from

    Returns:
        float:  yaw angle in [rads] (rotation about z-axis)
    """
    return math.atan2(2. * q.w * q.z, q.w ** 2 - q.z ** 2)

def yaw_to_quaternion(yaw: float) -> Quaternion:
    """ Convert a yaw rotation to quaternion

    Args:
        yaw (float): yaw angle in [rads] (rotation about z-axis)

    Returns:
        Quaternion: equivalent quaternion rotation
    """
    return Quaternion(
        z=math.sin(yaw / 2),
        w=math.cos(yaw / 2),
    )

def transform_to_state(t: Transform) -> TurtleBotState:
    """ Convert geometry_msgs.msg.Transform to asl_tb3_msgs.msg.TurtleBotState

    Args:
        t (Transform): transform

    Returns:
        TurtleBotState: state
    """
    return TurtleBotState(
        x=t.translation.x,
        y=t.translation.y,
        theta=quaternion_to_yaw(t.rotation),
    )

def state_to_transform(state: TurtleBotState) -> Transform:
    """ Convert asl_tb3_msgs.msg.TurtleBotState to geometry_msgs.msg.Transform

    Args:
        state (TurtleBotState): state

    Returns:
        Transform: transform
    """
    t = Transform()
    t.translation.x = state.x
    t.translation.y = state.y
    t.rotation = yaw_to_quaternion(state.theta)

    return t

def pose_to_state(p: Pose) -> TurtleBotState:
    """ Convert geometry_msgs.msg.Pose to asl_tb3_msgs.msg.TurtleBotState

    Args:
        p (Pose): pose

    Returns:
        TurtleBotState: state
    """
    return TurtleBotState(
        x=p.position.x,
        y=p.position.y,
        theta=quaternion_to_yaw(p.orientation),
    )

def state_to_pose(state: TurtleBotState) -> Pose:
    """ Convert asl_tb3_msgs.msg.TurtleBotState to geometry_msgs.msg.Pose

    Args:
        state (TurtleBotState): state

    Returns:
        Pose: pose
    """
    p = Pose()
    p.position.x = state.x
    p.position.y = state.y
    p.orientation = yaw_to_quaternion(state.theta)

    return p
