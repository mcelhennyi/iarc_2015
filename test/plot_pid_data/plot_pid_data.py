__author__ = 'odroid'
import numpy as np
import matplotlib.pyplot as pp
import random

ar = np.arange(300)
for x in range(1, 300):
    ar[x] = random.uniform(5, 20)

pp.plot(ar)
pp.show()