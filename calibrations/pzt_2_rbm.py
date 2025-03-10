import numpy as np
from scipy.io import savemat

data = np.load("pzt_2_rbm.pkl",allow_pickle=True)
k1 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T
data = np.load("pzt_f2d.pkl",allow_pickle=True)
k2 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T

k1p = k1[:,::2] - k1[:,1::2]
k2p = k2[::2,::2] - k2[1::2,1::2]
l = np.diag(np.diag(k2p))

rbm_2_pzt= {f"var{i}":l[i*3:(i+1)*3,i*3:(i+1)*3] @ np.linalg.inv(k1p[i*6+2:i*6+5,i*3:(i+1)*3])   for i in range(7)}
savemat("rbm_2_pzt.mat",rbm_2_pzt)
