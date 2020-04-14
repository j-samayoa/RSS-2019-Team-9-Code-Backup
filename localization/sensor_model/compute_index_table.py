import numpy as np
import pickle
from math import *
import scipy.stats


def prob(zt,zts):

    alpha_hit = 0.74
    alpha_short = 0.07
    alpha_max = 0.07
    alpha_rand = 0.12
    sigma = 0.5
    zt = float(zt)
    zts = float(zts)

    if 0.0 <= zt <= zmax:
        p_hit = scipy.stats.norm(zts,sigma).pdf(zt)
    else:
        p_hit = 0.0

    if 0.0 <= zt <= zts:
        p_short = (2.0/zts)*(1.0-zt/zts)
    else:
        p_short = 0.0

    if zt >= zmax:
        p_max = 1.0
    else:
        p_max = 0.0

    if 0.0 <= zt < zmax:
        p_rand = 1.0/zmax
    else:
        p_rand = 0.0

    p = alpha_hit*p_hit + alpha_short*p_short + alpha_max * p_max + alpha_rand*p_rand
    return p

# -----------------------------------------------------------

num_samples = 200
zmax = 100
ground_truth_distance = np.linspace(0.5 ,zmax,num_samples)#zt*
measured_distance = np.linspace(0.5,zmax,num_samples)#zt
# print(ground_truth_distance)

# sensor_d[zt_index][zt_star_index] = probability of (zt | zt_star) = P(measured | ground truth)
sensor_d = np.zeros((num_samples, num_samples))

for i, zt in enumerate(measured_distance):
    for j, zt_star in enumerate(ground_truth_distance):
        sensor_d[i][j] = prob(zt,zt_star)

with open('index_data.pkl', 'wb') as f:
    pickle.dump(sensor_d, f)

print('index table computed')
