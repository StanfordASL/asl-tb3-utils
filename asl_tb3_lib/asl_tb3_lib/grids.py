import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import typing as T


def snap_to_grid(data: np.ndarray, resolution: float) -> np.ndarray:
    return resolution * np.round(data / resolution)


# A 2D state space grid with a set of rectangular obstacles. The grid is fully deterministic
class DetOccupancyGrid2D(object):
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.obstacles = obstacles

    def is_free(self, x):
        for obs in self.obstacles:
            inside = True
            for dim in range(len(x)):
                if x[dim] < obs[0][dim] or x[dim] > obs[1][dim]:
                    inside = False
                    break
            if inside:
                return False
        return True

    def plot(self, fig_num=0):
        fig = plt.figure(fig_num)
        for obs in self.obstacles:
            ax = fig.add_subplot(111, aspect='equal')
            ax.add_patch(
            patches.Rectangle(
            obs[0],
            obs[1][0]-obs[0][0],
            obs[1][1]-obs[0][1],))

class StochOccupancyGrid2D(object):
    def __init__(self,
        resolution: float,
        size_xy: np.ndarray,
        origin_xy: np.ndarray,
        window_size: int,
        probs: T.Sequence[float],
        thresh: float = 0.5
    ) -> None:
        self.resolution = resolution
        self.size_xy = size_xy
        self.origin_xy = origin_xy
        self.probs = np.reshape(np.asarray(probs), (size_xy[1], size_xy[0]))
        self.window_size = window_size
        self.thresh = thresh

    def is_free(self, state_xy: np.ndarray) -> bool:
        # combine the probabilities of each cell by assuming independence
        # of each estimation
        state_xy = snap_to_grid(state_xy, self.resolution)
        grid_xy = ((state_xy - self.origin_xy) / self.resolution).astype(int)

        half_size = int(round((self.window_size-1)/2))
        grid_xy_lower = np.maximum(0, grid_xy - half_size)
        grid_xy_upper = np.minimum(self.size_xy, grid_xy + half_size + 1)

        prob_window = self.probs[grid_xy_lower[1]:grid_xy_upper[1],
                                 grid_xy_lower[0]:grid_xy_upper[0]]
        p_total = np.prod(1. - np.maximum(prob_window / 100., 0.))

        return (1. - p_total) < self.thresh

    def plot(self, fig_num=0):
        fig = plt.figure(fig_num)
        pts = []
        for i in range(self.probs.shape[0]):
            for j in range(self.probs.shape[1]):
                # convert i to (x,y)
                x = j * self.resolution + self.origin_x
                y = i * self.resolution + self.origin_y
                if not self.is_free((x,y)):
                    pts.append((x,y))
        pts_array = np.array(pts)
        plt.scatter(pts_array[:,0],pts_array[:,1],color="red",zorder=15,label='planning resolution')
        plt.xlim([self.origin_x, self.width * self.resolution + self.origin_x])
        plt.ylim([self.origin_y, self.height * self.resolution + self.origin_y])
