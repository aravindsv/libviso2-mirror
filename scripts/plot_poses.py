import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("stereo_file")
parser.add_argument("mono_file")
args = parser.parse_args()
stereo_file = args.stereo_file
mono_file = args.mono_file

plt.ion()

def getXYZ(file):
	x = file.readline().strip()
	if x == '' or x == '\n':
		return None, None, None
	else:
		x = x.split()[-1]
	y = file.readline().strip()
	if y == '' or x == '\n':
		return None, None, None
	else:
		y = y.split()[-1]
	z = file.readline().strip()
	if z == '' or x == '\n':
		return None, None, None
	else:
		z = z.split()[-1]
	file.readline()
	file.readline()
	return (float(x),float(y),float(z))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.show()

minx = -1
maxx = 1
miny = -1
maxy = 1
minz = -1
maxz = 1

with open(stereo_file, 'r') as fstereo:
	oldx, oldy, oldz = getXYZ(fstereo)
	#print("({},{})".format(oldx, oldy)) 
	
	x,y,z = getXYZ(fstereo)
	#print("({},{})".format(x, y)) 
	while x is not None:
		minx = min(minx, x)
		miny = min(miny, y)
		minz = min(minz, z)
		maxx = max(maxx, x)
		maxy = max(maxy, y)
		maxz = max(maxz, z)

		ax.set_xlim(minx-1, maxx+1)
		ax.set_ylim(miny-1, maxy+1)
		ax.set_zlim(minz-1, maxz+1)

		ax.plot([oldx, x], [oldy, y], [oldz, z], 'b')

		# plt.subplot(1,2,1)
		# plt.xlim(minx-1, maxx+1)
		# plt.ylim(miny-1, maxy+1)
		# plt.plot([oldx, x], [oldy,y], 'b')

		# plt.subplot(1,2,2)
		# plt.xlim(minx-1, maxx+1)
		# plt.ylim(minz-1, maxz+1)
		# plt.plot([oldx, x], [oldz,z], 'b')

		plt.pause(0.03)
		oldx, oldy, oldz = x,y,z
		x,y,z = getXYZ(fstereo)
		#print("({},{})".format(x, y)) 

print("Finished plotting stereo trajectory. Moving on to mono")

with open(mono_file, 'r') as fmono:
	oldx, oldy, oldz = getXYZ(fmono)
	#print("({},{})".format(oldx, oldy)) 
	
	x,y,z = getXYZ(fmono)
	#print("({},{})".format(x, y)) 
	while x is not None:
		minx = min(minx, x)
		miny = min(miny, y)
		minz = min(minz, z)
		maxx = max(maxx, x)
		maxy = max(maxy, y)
		maxz = max(maxz, z)

		ax.set_xlim(minx-1, maxx+1)
		ax.set_ylim(miny-1, maxy+1)
		ax.set_zlim(minz-1, maxz+1)

		ax.plot([oldx, x], [oldy, y], [oldz, z], 'r')

		# plt.subplot(1,2,1)
		# plt.xlim(minx-1, maxx+1)
		# plt.ylim(miny-1, maxy+1)
		# plt.plot([oldx, x], [oldy,y], 'r')

		# plt.subplot(1,2,2)
		# plt.xlim(minx-1, maxx+1)
		# plt.ylim(minz-1, maxz+1)
		# plt.plot([oldx, x], [oldz,z], 'r')

		plt.pause(0.03)
		oldx, oldy, oldz = x,y,z
		x,y,z = getXYZ(fmono)
		#print("({},{})".format(x, y)) 

print("Finished plotting mono trajectory")
plt.show(block=True)

		# for line1, line2 in zip(f1.readlines(), f2.readlines()):
		# 	if i == 3:
		# 		print("({},{}) and ({},{})".format(x1, y1, x2, y2))
		# 		plt.xlim(-80,80)
		# 		plt.ylim(-50,50)
		# 		# plt.plot([old_x1, x1], [old_y1, y1], 'r')
		# 		plt.plot([old_x2, x2], [old_y2, y2], 'b')
		# 		plt.pause(0.03)
		# 		old_x1 = x1
		# 		old_x2 = x2
		# 		old_y1 = y1 # 		old_y2 = y2
		# 		old_z1 = z1
		# 		old_z2 = z2
		# 		i = 0
		# 	else:
		# 		if i == 0:
		# 			x1 = line1.strip().split()[-1]
		# 			x2 = line2.strip().split()[-1]
		# 		elif i == 1:
		# 			y1 = line1.strip().split()[-1]
		# 			y2 = line2.strip().split()[-1]
		# 		elif i == 2:
		# 			z1 = line1.strip().split()[-1]
		# 			z2 = line2.strip().split()[-1]

		# 		i += 1
