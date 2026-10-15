import numpy as np
import matplotlib.pyplot as plt

def find_mean_offset(A: np.ndarray, B: np.ndarray):
    dist_hat = A-B
    return np.mean(dist_hat, 0), np.std(dist_hat, 0)

pA = np.array([[375.618, -1008.882, 483.050],
                [656.288, -1156.190, 459.250],
                [51.881, -808.349, 218.804]])
pB = np.array([[307.442, 1221.726, 520.420],
                [585.414, 1076.776, 505.247],
                [-19.518, 1437.612, 252.559]])
mean, stdev = find_mean_offset(pA, pB)
plt.plot(pA[0], pA[1], color="blue")
plt.plot(pB[0], pB[1], color = "green")
plt.show()

with (np.printoptions(precision=2, sign="-")):
    print(f'Found a mean offset of {mean} with standard deviation {stdev}')


