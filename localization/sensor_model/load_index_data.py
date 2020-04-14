import pickle
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np

with open('index_data.pkl', 'rb') as f:
    lookup = pickle.load(f)

print('table size:', len(lookup))
print(lookup[199][199])


fig = plt.figure()
#ax = plt.axes(projection='3d')
ax = fig.add_subplot(111,projection = '3d')

#ax.contour3D(measured_distance,ground_truth_distance,z,100,cmap = 'Reds')

ground_truth_distance = np.linspace(0.5,100,200) #zt*
measured_distance = np.linspace(0.5,100,200) #zt

X,Y = np.meshgrid(ground_truth_distance,measured_distance,)
ax.plot_wireframe(X,Y,lookup)
ax.set_xlabel('zt*')
ax.set_ylabel('zt')
ax.set_zlabel('P(Measured Distance | Ground Truth')
ax.set_zlim(-.1,1)
plt.show()
