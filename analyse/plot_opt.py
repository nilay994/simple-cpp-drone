import os
import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

opt_file_flag = False
opt_file = "../remote/build/states.csv"
if os.path.isfile(opt_file):
	if os.stat(opt_file) != 0:
		opt = np.genfromtxt(opt_file, delimiter=",", skip_footer=1)
		opt_file_flag = True

fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')

if opt_file_flag == True:
	ax.plot(opt[:,2], opt[:,4], '.')
	ax.set_aspect('equal')
	ax.set(xlim=(-3, 3), ylim=(-3, 3))

plt.show()

