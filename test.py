from scipy import optimize, math
from numpy.linalg import norm
import numpy as np
import matplotlib.pyplot as plt

line_x = [75522.07977, 75562.18182, 75585.56548, 75591.91457, 75579.54545, 75547.2155, 75493.33333]
line_y = [23752.49288, 24095.63636, 24485.49107, 24927.91762, 25429.85893, 26000.40355, 26648.75]
A, B = optimize.curve_fit(lambda x, A, B: A * x + B, line_x, line_y)[0]

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(line_x, line_y)
ax.plot([75600,line_x[-1]], [A * 75600 + B, A*line_x[-1] + B], c='red')
plt.show()
