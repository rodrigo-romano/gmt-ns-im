#!/usr/bin/env python
# coding: utf-8

# In[1]:


import os
import sys
sys.path.append('/home/ubuntu/CEO/python')
import ceo
from ceo import Mapping,mapping
import pandas as pd
import numpy as np
from scipy.io import loadmat
import matplotlib.pyplot as plt
import matplotlib.tri as tri
get_ipython().run_line_magic('matplotlib', 'inline')


# In[2]:


root = "/home/ubuntu/mnt/20230530_1756_zen_30_M1_202110_FSM_202305_Mount_202305_noStairs"
last_part = root.split('/')[-1]
date_time = last_part.split('_')[:2]
fem_id = '_'.join(date_time)
fem_id


# In[3]:


data = np.load("m1_singular_modes.pkl", allow_pickle=True)
data[0].keys()


# In[4]:


[x['shape'] for x in data]


# In[5]:


np.asarray(data[0]["modes"]).shape,602*(335-6)


# In[6]:


def geomshow(sid):
    i = sid - 1
    xyz = np.asarray(data[i]["mode_nodes"])
    x = xyz[:,0].ravel()
    y = xyz[:,1].ravel()
    triang = tri.Triangulation(x, y)
    if sid==7:
        triang.set_mask(np.hypot(x[triang.triangles].mean(axis=1),
                 y[triang.triangles].mean(axis=1))
        < 1.2)
    xyz_a = np.asarray(data[i]["actuator_nodes"])
    xa = xyz_a[:,0].ravel()
    ya = xyz_a[:,1].ravel()
    fig,ax = plt.subplots()
    ax.triplot(triang)
    ax.plot(xa,ya,'k.')
    ax.set_aspect('equal')
    ax.grid()


# In[7]:


geomshow(1)


# In[8]:


geomshow(7)


# In[9]:


def modeshow(sid):
    i = sid - 1
    D = 8.5
    xyz = np.asarray(data[i]["mode_nodes"])
    x = xyz[:,0].ravel()/D
    y = xyz[:,1].ravel()/D
    fig,ax = plt.subplots(figsize=[18,15])
    n = 7
    (r,c) = data[i]["shape"]
    Mn = np.asarray(data[i]["modes"],order="F").reshape(c-6,r).T
    vmin = np.min(Mn[:,:n*n])
    vmax = np.max(Mn[:,:n*n])
    k = 0
    for i in range(n):
        for j in range(n):
            triang = tri.Triangulation(x+j, y+i)
            if sid==7:
                triang.set_mask(np.hypot(x[triang.triangles].mean(axis=1),
                         y[triang.triangles].mean(axis=1))
                < 1.2/D)
            h = ax.tripcolor(triang,Mn[:,k],vmin=vmin,vmax=vmax)
            k += 1
            ax.text(j+0.35,i+0.35,f'{k}',fontsize='small')
    ax.set_aspect('equal')
    ax.grid()
    fig.colorbar(h,ax=ax)


# In[10]:


def rawmodeshow(sid):
    i = sid - 1
    D = 8.5
    xyz = np.asarray(data[i]["mode_nodes"])
    x = xyz[:,0].ravel()/D
    y = xyz[:,1].ravel()/D
    fig,ax = plt.subplots(figsize=[18,15])
    n = 7
    (r,c) = data[i]["shape"]
    Mn = np.asarray(data[i]["raw_modes"],order="F").reshape(c,r).T
    vmin = np.min(Mn[:,:n*n])
    vmax = np.max(Mn[:,:n*n])
    k = 0
    for i in range(n):
        for j in range(n):
            triang = tri.Triangulation(x+j, y+i)
            if sid==7:
                triang.set_mask(np.hypot(x[triang.triangles].mean(axis=1),
                         y[triang.triangles].mean(axis=1))
                < 1.2/D)
            h = ax.tripcolor(triang,Mn[:,k],vmin=vmin,vmax=vmax)
            k += 1
            ax.text(j+0.35,i+0.35,f'{k}',fontsize='small')
    ax.set_aspect('equal')
    ax.grid()
    fig.colorbar(h,ax=ax)


# In[11]:


modeshow(1)


# In[12]:


modeshow(7)


# In[13]:


rawmodeshow(1)


# In[14]:


rawmodeshow(7)


# In[15]:


from scipy.io import savemat
modes = {f"B2F_{i}":[] for i in range(1,8)}
for i in range(7):
    (r,c) = data[i]["shape"]
    M = np.asarray(data[i]["mode_2_force"],order="F").reshape(c-6,c).T
    modes[f"B2F_{i+1}"] = M
savemat(f"{fem_id}_m1_mode_to_force.mat",modes)


# In[16]:


n_mode =27
i = 0
xyz = np.asarray(data[i]["mode_nodes"])
(r,c) = data[i]["shape"]
B = np.asarray(data[i]["modes"],order="F").reshape(c-6,r).T
bm = Mapping(xy=xyz[:,:2],z=B[:,:n_mode])
bm = bm(256,8.5)
bm.suit

s2b = [0,0,0,0,0,0,0]
for i in range(1,7):
    xyz = np.asarray(data[i]["mode_nodes"])
    (r,c) = data[i]["shape"]
    B = np.asarray(data[i]["modes"],order="F").reshape(c-6,r).T
    bm_i = Mapping(xy=xyz[:,:2],z=B[:,:n_mode])
    bm_i = bm_i(256,8.5)
    bm_i.suit
    s2b[i] =i
    bm = mapping.cat(bm,bm_i,s2b)
bm.suit

bm.suit['M'].shape,256*256*7*27

bm.dump(f"{fem_id}_m1_bending_modes")


# In[17]:


i = 0
xyz = np.asarray(data[i]["mode_nodes"])
(r,c) = data[i]["shape"]
B = np.asarray(data[i]["raw_modes"],order="F").reshape(c,r).T
bm = Mapping(xy=xyz[:,:2],z=B)
bm = bm(256,8.5)
bm.suit

s2b = [0,0,0,0,0,0,0]
for i in range(1,7):
    xyz = np.asarray(data[i]["mode_nodes"])
    (r,c) = data[i]["shape"]
    B = np.asarray(data[i]["raw_modes"],order="F").reshape(c,r).T
    if i==6:
        B = np.hstack([B,np.zeros((579,335-306))])
    bm_i = Mapping(xy=xyz[:,:2],z=B)
    bm_i = bm_i(256,8.5)
    bm_i.suit
    s2b[i] =i
    bm = mapping.cat(bm,bm_i,s2b)
bm.suit

bm.suit['M'].shape,256*256*7*27

bm.dump(f"{fem_id}_m1_raw_bending_modes")


# In[18]:


bm.suit


# In[19]:


bm.suit['M'].shape,256*256*7*335


# In[20]:


gmt = ceo.GMT_MX(M1_mirror_modes=f"{fem_id}_m1_bending_modes",M1_N_MODE=27)
src = ceo.Source("V",rays_box_size=25.5,rays_box_sampling=512)
src>>(gmt,)


# In[21]:


~gmt
state = gmt.state
state["M1"]["modes"][:,0] = 1e-5
gmt^=state
+src
fig,ax = plt.subplots()
h = ax.imshow(src.phase.host()*1e9,origin="lower")
fig.colorbar(h,ax=ax)

