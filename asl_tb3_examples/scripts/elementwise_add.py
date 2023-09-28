#!/usr/bin/env python3

from asl_tb3_examples.timer import timed

import numpy as np
import typing as T


def get_random_array(size: T.Tuple[int, ...]) -> T.List[float]:
    return np.random.random(size=size)

def add_loop(arr1: np.ndarray, arr2: np.ndarray) -> float:
    out = np.zeros_like(arr1)
    for i in range(arr1.shape[0]):
        for j in range(arr1.shape[1]):
            out[i, j] = arr1[i, j] + arr2[i, j]

    return out

def add_vectorized(arr1: np.ndarray, arr2: np.ndarray) -> float:
    return arr1 + arr2


if __name__ == "__main__":
    rand_arr1 = get_random_array((1000, 1000))
    rand_arr2 = get_random_array((1000, 1000))
    result_loop = timed(add_loop, arr1=rand_arr1, arr2=rand_arr2)
    result_vectorized = timed(add_vectorized, arr1=rand_arr1, arr2=rand_arr2)

    print(f"Element-wise add with for loop: {result_loop}")
    print(f"Element-wise add with numpy vectorization: {result_vectorized}")
