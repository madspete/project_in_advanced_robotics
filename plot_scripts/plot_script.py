#!/usr/bin/python
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import csv
import sys
import math

fig, ax = plt.subplots()
forceZ = []
forceX = []
forceY = []
amount = []
amountf = []

path = "/home/marcus/project/project_in_advanced_robotics/wrenchCurve.csv"
pathf = "/home/marcus/project/project_in_advanced_robotics/forcesCurved.txt"

with open(path) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	counter = 0.0
	step = 0.002
	for row in csv_reader:
		forceZ.append(float(row[4])*-1.0)
		forceX.append(float(row[0]))
		amount.append(counter)
		counter += step

with open(pathf) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	counterf = 0.0
	step = 0.002
	for row in csv_reader:
		forceY.append(float(row[0]))
		amountf.append(counterf)
		counterf += step
		
ax.plot(amount, forceZ, color="blue", marker=" ")
ax.plot(amountf, forceY, color="green", marker=" ")
#ax.plot(amount, forceX, color="red", marker=" ")
ax.set_xlabel('Time [s]', fontsize=14)
ax.set_ylabel('Contact force [N]', fontsize=14)
x_patch = mpatches.Patch(color='blue', label='Measured contact force')
y_patch = mpatches.Patch(color='green', label='Target force')
#y_patch = mpatches.Patch(color='green', label='Y')
#z_patch = mpatches.Patch(color='blue', label='Z')
plt.legend(loc="upper center", bbox_to_anchor=(0.5, 1.15), ncol=3, handles=[x_patch, y_patch])
#plt.ylim([0,20])
MSE = np.square(np.subtract(forceZ,forceY[:len(forceZ)])).mean()
RMSE = math.sqrt(MSE)
print(RMSE)
plt.show()