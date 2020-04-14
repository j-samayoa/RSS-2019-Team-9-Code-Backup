import json
import pickle

pkl_file = open('data.pkl','rb')
sensor_d = pickle.load(pkl_file)
print(sensor_d[(30,40.5)])
