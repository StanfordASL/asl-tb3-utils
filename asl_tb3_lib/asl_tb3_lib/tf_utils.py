import math

from geometry_msgs.msg import Quaternion


def quaternion_to_yaw(q: Quaternion) -> float:
    return math.atan2(2. * q.w * q.z, q.w ** 2 - q.z ** 2)
