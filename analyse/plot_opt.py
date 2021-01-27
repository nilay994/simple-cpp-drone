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
ax1 = fig.add_subplot(211, aspect='equal')
ax2 = fig.add_subplot(212)

if opt_file_flag == True:
	ax1.plot(opt[:,2], opt[:,4], '.', label="top view")
	ax1.set_xlabel("x (m)")
	ax1.set_ylabel("y (m)")
	ax1.set_aspect('equal')
	ax1.set(xlim=(-3, 3), ylim=(-3, 3))
	ax1.legend()

	ax2.plot(opt[:,1], '.', label = "$v_x^w$")
	ax2.plot(opt[:,3], '.', label = "$v_y^w$")
	ax2.set_ylabel("t (s)")
	ax2.set_xlabel("v (m/s)")
	ax2.legend()

plt.show()

