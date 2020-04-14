from prob import *
from mpl_toolkits import mplot3d
import numpy as np
import pickle
import matplotlib.pyplot as plt

num_samples = 75
ground_truth_distance = np.linspace(0.1,11,num_samples)#zt*
measured_distance = np.linspace(0.0,11,num_samples)#zt
print(ground_truth_distance)
sensor_d = dict()

#z = np.zeros((num_samples,num_samples))
Z = []

for i, zt in enumerate(measured_distance):
    temp = []
    for j, zt_star in enumerate(ground_truth_distance):
        key = (zt,zt_star)
        val = prob(zt,zt_star)
        sensor_d[key] = val
        temp.append(val)
        #z[j,i]=val
        #if abs(val) < 0.1:
            #print("j: ",j)
            #print("i: ",i)
            #print("-------------------\n")
       #print("zt* = ",zt_star, ", zt = ", zt, ", z = ", val)
    Z.append(temp)
    temp = []



output = open('data.pkl', 'wb')
pickle.dump(sensor_d,output)

fig = plt.figure()
#ax = plt.axes(projection='3d')
ax = fig.add_subplot(111,projection = '3d')

#ax.contour3D(measured_distance,ground_truth_distance,z,100,cmap = 'Reds')

X,Y = np.meshgrid(ground_truth_distance,measured_distance,)
ax.plot_wireframe(X,Y,Z)
ax.set_xlabel('zt*')
ax.set_ylabel('zt')
ax.set_zlabel('P(Measured Distance | Ground Truth')
ax.set_zlim(-.1,1)
plt.show()


