#!/usr/bin/env python3

from asl_tb3_examples.timer import timed

import numpy as np
import typing as T


def get_random_array(size: int) -> T.List[float]:
    return np.random.random(size=size)

def self_outer_loop(vec: np.ndarray) -> float:
    x = np.zeros((vec.shape[0], vec.shape[0]))
    for i in range(vec.shape[0]):
        for j in range(vec.shape[0]):
            x[i, j] = vec[i] * vec[j]

    return x

def self_outer_vectorized(vec: np.ndarray) -> float:
    return vec[None] * vec[:, None]


if __name__ == "__main__":
    rand_vec = get_random_array(1000)
    result_loop = timed(self_outer_loop, vec=rand_vec)
    result_vectorized = timed(self_outer_vectorized, vec=rand_vec)

    print(f"Outer product with for loop:\n{result_loop}")
    print(f"Outer product with numpy vectorization:\n{result_vectorized}")
