import math
import numpy as np
import typing as T

from asl_tb3_msgs.msg import TurtleBotState

def wrap_angle(x: T.Union[float, np.ndarray]) -> T.Union[float, np.ndarray]:
    """ Wrap angle (radians) to [-pi, pi]

    Args:
        x (T.Union[float, np.ndarray]): angle to be wrapped

    Returns:
        T.Union[float, np.ndarray]: equivalent angle in [-pi, pi]
    """
    return (x + math.pi) % (2 * math.pi) - math.pi

def distance_linear(s1: TurtleBotState, s2: TurtleBotState) -> float:
    """ Computer linear distance between two states

    Args:
        s1 (TurtleBotState): s1
        s2 (TurtleBotState): s2

    Returns:
        float:  linear distance in [m]
    """
    return math.sqrt((s1.x - s2.x) ** 2 + (s1.y - s2.y) ** 2)

def distance_angular(s1: TurtleBotState, s2: TurtleBotState) -> float:
    """ Compute angular distance between two states

    Args:
        s1 (TurtleBotState): s1
        s2 (TurtleBotState): s2

    Returns:
        float:  angular distance in [0, pi] rads
    """
    return abs(wrap_angle(s1.theta - s2.theta))
