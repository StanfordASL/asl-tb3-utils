import time
import typing as T

def timed(func: T.Callable, *args, **kwargs):
    t0 = time.time()
    result = func(*args, **kwargs)
    t1 = time.time()

    print(f"Function {func.__name__} took {t1 - t0:.4f}s")

    return result
