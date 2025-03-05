import numpy as np


data = np.load("pzt_2_rbm.pkl",allow_pickle=True)
k1 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T
data = np.load("pzt_f2d.pkl",allow_pickle=True)
k2 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T

k1p = k1[:,::2] - k1[:,1::2]
k2p = k2[::2,::2] - k2[1::2,1::2]
l = np.diag(1/np.diag(k2p))

pzt_d_2_rbm = {f"var{i}":k1p[i*6+3:(i+1)*6,i*3:(i+1)*3] @ l[i*3:(i+1)*3,i*3:(i+1)*3] for i in range(7)}

