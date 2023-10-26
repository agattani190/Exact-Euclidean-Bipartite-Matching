from decimal import Decimal
import time
import numpy as np
with open('costMatrix') as file:
    cost = [[float(digit) for digit in line.split()] for line in file]
from scipy.optimize import linear_sum_assignment

startTime = time.time()
row_ind, col_ind = linear_sum_assignment(cost)
cost = np.array(cost)
endTime = time.time()
print(cost[row_ind, col_ind].sum())
l = min(len(row_ind), len(col_ind))
for i in range(l):
    print(row_ind[i], ":", col_ind[i])
print("Time taken: ", (endTime - startTime) * 1000)
