#!/usr/bin/env python3

from asl_tb3_examples.timer import timed

import numpy as np
import typing as T


def get_random_array(size: int) -> T.List[float]:
    return np.random.random(size=size)

def self_dot_loop(vec: np.ndarray) -> float:
    x = 0
    for i in range(vec.shape[0]):
        x += vec[i]**2

    return x

def self_dot_vectorized(vec: np.ndarray) -> float:
    return vec.T @ vec


if __name__ == "__main__":
    rand_vec = get_random_array(1000000)
    result_loop = timed(self_dot_loop, vec=rand_vec)
    result_vectorized = timed(self_dot_vectorized, vec=rand_vec)

    print(f"Dot product with for loop: {result_loop}")
    print(f"Dot product with numpy vectorization: {result_vectorized}")
