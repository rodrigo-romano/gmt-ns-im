import numpy as np
from scipy.io import loadmat, savemat

data = np.load("hardpoints_2_rbm.pkl",allow_pickle=True)
k1 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T
data = np.load("hardpoints_2_edge-sensors.pkl",allow_pickle=True)
k2 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T
data = loadmat("M1_edge_sensor_conversion.mat")
A1 = data['A1']
k2p = A1@k2
m1_r_es = np.linalg.lstsq(k2p[:,:36].T,k1[:36,:36].T,rcond=None)[0].T
m1_r_es = np.vstack([m1_r_es,np.zeros((6,48))]) @ A1

savemat("es_2_rbm.mat",{"m1_r_es":m1_r_es})


