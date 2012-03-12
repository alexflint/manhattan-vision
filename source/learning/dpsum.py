import numpy as np

def n(A, y, x, t):
    if (x == 0):
        return int(A[y,x] >= t)

def s(A, y, x, t):
    if (x == 0):
        return A[y,x] * int(A[y,x] >= t)
    else:
        r = 0
        for yy in range(max(y-1, 0), min(y+2, A.shape[0])):
            r += s(A, yy, x-1, t-A[y,x]) + n(A, yy, x-1, t-A[y,x])*A[y,x]
        return r

def sum(A, t):
    r = 0
    for y in range(A.shape[0]):
        r += s(A, y, A.shape[1]-1, t)
    return r
