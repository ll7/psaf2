from numpy import array, linspace
from sklearn.neighbors import KernelDensity
import matplotlib.pyplot as plt
from scipy.signal import argrelextrema
import numpy as np


testdata = [0.8338184368430228, -0.8695364039286193, -0.8897793163477786, 0.8531675277967423, 0.3656023317069669, -0.8638882660347803, 0.8341400147073799, 0.7853981633974483, -0.844153986113171, 0.7853981633974483, 0.7853981633974483, 0.8012698463892384, 0.34555558058171215, 0.8323435116588559, -0.870903457075653, -0.8788749445560378, 0.3028848683749714, -0.8519663271732721, 0.7853981633974483, 0.2860514417173182, 0.4844779290370232, -0.8960553845713439, 0.2914567944778671, 0.3217505543966422, 0.37089128881266237, 0.8076167287241673, 0.0, 0.2914567944778671, -0.844153986113171, 0.8567056281827387]

resolution = 10000
bandwidth = 0.001

pos_angles = [np.pi + v if v < 0 else v for v in testdata]

a = array(pos_angles).reshape(-1, 1)
kde = KernelDensity(kernel='gaussian', bandwidth=bandwidth).fit(a)
s = linspace(0, 2 * np.pi, resolution)
e = kde.score_samples(s.reshape(-1,1))

mi, ma = argrelextrema(e, np.less), argrelextrema(e, np.greater)

print(a)

clusters = []
clusters.append(a[a < s[mi][0]])

for i in range(0, len(mi[0]) - 1):
    clusters.append(a[(a >= s[mi][i]) * (a <= s[mi][i+1])])
clusters.append(a[a >= s[mi][len(mi[0]) - 1]])

print(clusters)
