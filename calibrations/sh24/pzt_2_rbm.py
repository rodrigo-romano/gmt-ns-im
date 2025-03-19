import numpy as np
from scipy.io import savemat

data = np.load("pzt_2_rbm.pkl",allow_pickle=True)
k1 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T
data = np.load("pzt_f2d.pkl",allow_pickle=True)
k2 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T

k1p = k1[:,::2] - k1[:,1::2]
k2p = k2[::2,::2] - k2[1::2,1::2]
l = np.diag(np.diag(k2p))

rbm_2_pzt= {f"var{i}":l[i*3:(i+1)*3,i*3:(i+1)*3] @ np.linalg.pinv(k1p[i*6+2:i*6+5,i*3:(i+1)*3])   for i in range(7)}
savemat("rbm_2_pzt_rco.mat",rbm_2_pzt)

V = np.vstack([np.array([0,-2/np.sqrt(6)]),np.array([-1/np.sqrt(2),1/np.sqrt(6)]),np.array([1/np.sqrt(2),1/np.sqrt(6)])])
l = np.diag(1/np.diag(k2p))
T = [ k1p[i*6+3:i*6+5,i*3:(i+1)*3]@l[i*3:(i+1)*3,i*3:(i+1)*3]@V   for i in range(7)]
rbm_2_pzt= {f"var{i}":V@np.linalg.inv(x)   for (i,x) in enumerate(T)}
savemat("rbm_2_pzt_pth.mat",rbm_2_pzt)


# PZT displacement to M2 RBM
O = np.kron(np.eye(3),[1,-1])
m2_pzt_r = {f"var{i}": k1p[i*6:i*6+6,i*3:(i+1)*3]@l[i*3:(i+1)*3,i*3:(i+1)*3]@O   for i in range(7)}
savemat("m2_pzt_r.mat",m2_pzt_r)
