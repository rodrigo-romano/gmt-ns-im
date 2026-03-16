from pprint import pprint
import h5py
import numpy as np
import ceo
import matplotlib.pyplot as plt


def load_and_map(fn, diam, npix):
    item = fn["/dataset"]
    map1 = np.array(item).T  # in micron
    fig, axs = plt.subplots(ncols=2, figsize=(12, 5))
    h0 = axs[0].imshow(map1)
    # plt.title('m')
    # map1[100:150,500:510] = 1  # fiducial marks to verify segment rotation
    # map1[500:550,500:510] = 1
    fig.colorbar(h0, ax=axs[0], label="Surface error [micron]")
    print(np.nanstd(map1))

    # for i in range(len(item.attrs)): print item.attrs.keys()[i], item.attrs.values()[i]

    pixelSize = (item.attrs.get("pixelSize"))[0]
    centerRow = (item.attrs.get("centerRow"))[0]
    centerCol = (item.attrs.get("centerCol"))[0]

    print(item.attrs["units"])

    ### Assign polar coordinates to map.
    OD = 8.405  # Outer Diameter
    ucDiamInPix = OD / pixelSize

    [rows, cols] = np.shape(map1)
    xVec = np.linspace(1, cols, cols)
    xVec = (xVec - centerCol) * pixelSize  # m
    yVec = np.linspace(1, rows, rows)
    yVec = (centerRow - yVec) * pixelSize  # m, increasing upward

    [x, y] = np.meshgrid(xVec, yVec)  # rows x cols
    r = np.hypot(x, y)
    xy = np.vstack([x.flatten(), y.flatten()]).T
    z = map1.reshape(-1, 1) * 1e-6
    S = ceo.Mapping(xy, z)
    S = S(npix, diam)

    h1 = axs[1].imshow(S.suit["M"].reshape(npix, npix))
    fig.colorbar(h1, ax=axs[1])

    return S


D = 8.5
n_px = 256
fem_id = "20250506_1715"
root = "/home/rconan/Documents/GMT/Notes/M1/M1_polishing_error/Data/"
S1 = load_and_map(
    h5py.File(root + "120814-15 GMT1 corrected avg_CCv0001.h5", "r"), D, n_px
)
S2 = load_and_map(h5py.File(root + "2019.06/GMT2 avg stitched.h5", "r"), D, n_px)

# S = ceo.mapping.cat(S1, S2, [0] + 6 * [1])
S = ceo.Mapping()
S.suit["Ni"] = S1.suit["Ni"]
S.suit["L"] = S1.suit["L"]

S.suit["N_SET"] = np.array(7, dtype=np.int32)
S.suit["N_MODE"] = S1.suit["N_MODE"]
S.suit["s2b"] = np.array(range(7), dtype=np.int32)
M = np.hstack(
    [
        S1.suit["M"],
        S2.suit["M"],
        S2.suit["M"],
        S2.suit["M"],
        S2.suit["M"],
        S2.suit["M"],
        S2.suit["M"],
    ]
)
S.suit["M"] = M.flatten(order="F")
pprint(S.suit)

m1_raw = ceo.Mapping()
m1_raw.load(f"{fem_id}_m1_raw_bending_modes")
pprint(m1_raw.suit)

m1_raw_polish = m1_raw + S
pprint(m1_raw_polish.suit)
m1_raw_polish.dump(f"{fem_id}_m1_raw_bending_modes_polish12")
