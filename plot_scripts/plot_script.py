#!/usr/bin/python
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import csv
import sys

fig, ax = plt.subplots()
forceZ = []
forceX = []
forceY = []
amount = []

path = "/home/mads/git/project_in_advanced_robotics/trajectory_learning/test.csv"

with open(path) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	counter = 0
	for row in csv_reader:
		forceZ.append(float(row[2]))
		forceY.append(float(row[1]))
		forceX.append(float(row[0]))
		amount.append(counter)
		counter += 1
		
#ax.plot(amount, forceZ, color="blue", marker=" ")
ax.plot(amount, forceY, color="green", marker=" ")
ax.plot(amount, forceX, color="red", marker=" ")
ax.set_xlabel('Force Point', fontsize=14)
ax.set_ylabel('Force', fontsize=14)
x_patch = mpatches.Patch(color='red', label='X')
#y_patch = mpatches.Patch(color='green', label='Y')
#z_patch = mpatches.Patch(color='blue', label='Z')
#plt.legend(loc="upper center", bbox_to_anchor=(0.5, 1.15), ncol=3, handles=[x_patch, y_patch, z_patch])

plt.show()