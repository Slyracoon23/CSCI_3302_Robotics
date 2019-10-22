import numpy as np
from matplotlib import colors
import time
import matplotlib.pyplot as plt

map_array = np.random.rand(10,10) * 20

plt.ion()
plt.show()
plt.clf()

cmap = colors.ListedColormap(['blue', 'red'])
bounds=[0,0.5,1]

norm = colors.BoundaryNorm(bounds, cmap.N)
fig, ax = plt.subplots()
ax.imshow(map_array, interpolation='nearest', origin='lower',        cmap=cmap, norm=norm)

ax.grid(which='major', axis='both', linestyle='-', color='k',        linewidth=2)
ax.set_xticks(np.arange(0,10,1))
ax.set_yticks(np.arange(0,10,1))

plt.draw()
plt.pause(0.001)
input()

