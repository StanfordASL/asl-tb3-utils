import numpy as np
import typing as T


def snap_to_grid(state: np.ndarray, resolution: float) -> np.ndarray:
    """ Snap continuous coordinates to a finite-resolution grid

    Args:
        state (np.ndarray): a size-2 numpy array specifying the (x, y) coordinates
        resolution (float): resolution of the grid

    Returns:
        np.ndarray: state-vector snapped onto the specified grid
    """
    return resolution * np.round(state / resolution)


class StochOccupancyGrid2D(object):
    """ A stochastic occupancy grid derived from ROS2 map data

    The probability of grid cell being occupied is computed by the joint probability of
    any neighboring cell being occupied within some fixed window. For some examples of size-3
    occupancy windows,

    0.1 0.1 0.1
    0.1 0.1 0.1  ->  1 - (1 - 0.1)**9 ~= 0.61
    0.1 0.1 0.1

    0.0 0.1 0.0
    0.0 0.1 0.0  ->  1 - (1 - 0)**6 * (1 - 0.1)**3 ~= 0.27
    0.0 0.1 0.0

    The final occupancy probability is then converted to binary occupancy using a threshold
    """

    def __init__(self,
        resolution: float,
        size_xy: np.ndarray,
        origin_xy: np.ndarray,
        window_size: int,
        probs: T.Sequence[float],
        thresh: float = 0.5
    ) -> None:
        """
        Args:
            resolution (float): resolution of the map
            size_xy (np.ndarray): size-2 integer array representing map size
            origin_xy (np.ndarray): size-2 float array representing map origin coordinates
            window_size (int): window size for computing occupancy probabilities
            probs (T.Sequence[float]): map data
            thresh (float): threshold for final binarization of occupancy probabilites
        """
        self.resolution = resolution
        self.size_xy = size_xy
        self.origin_xy = origin_xy
        self.probs = np.reshape(np.asarray(probs), (size_xy[1], size_xy[0]))
        self.window_size = window_size
        self.thresh = thresh

    def state2grid(self, state_xy: np.ndarray) -> np.ndarray:
        """ convert real state coordinates to integer grid indices

        Args:
            state_xy (np.ndarray): real state coordinates (x, y)

        Returns:
            np.ndarray: quantized 2D grid indices (kx, ky)
        """
        state_snapped_xy = snap_to_grid(state_xy, self.resolution)
        grid_xy = ((state_snapped_xy - self.origin_xy) / self.resolution).astype(int)

        return grid_xy

    def grid2state(self, grid_xy: np.ndarray) -> np.ndarray:
        """ convert integer grid indices to real state coordinates

        Args:
            grid_xy (np.ndarray): integer grid indices (kx, ky)

        Returns:
            np.ndarray: real state coordinates (x, y)
        """
        return (grid_xy * self.resolution + self.origin_xy).astype(float)

    def is_free(self, state_xy: np.ndarray) -> bool:
        """ Check whether a state is free or occupied

        Args:
            state_xy (np.ndarray): size-2 state vectory of (x, y) coordinate

        Returns:
            bool: True if free, False if occupied
        """
        # combine the probabilities of each cell by assuming independence of each estimation
        grid_xy = self.state2grid(state_xy)

        half_size = int(round((self.window_size-1)/2))
        grid_xy_lower = np.maximum(0, grid_xy - half_size)
        grid_xy_upper = np.minimum(self.size_xy, grid_xy + half_size + 1)

        prob_window = self.probs[grid_xy_lower[1]:grid_xy_upper[1],
                                 grid_xy_lower[0]:grid_xy_upper[0]]
        p_total = np.prod(1. - np.maximum(prob_window / 100., 0.))

        return (1. - p_total) < self.thresh

