#!/usr/bin/python
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import csv
import sys

fig, ax = plt.subplots()
amount = []
oriw = []
orix = []
oriy = []
oriz = []

amount1 = []
oriw2 = []
orix2 = []
oriy2 = []
oriz2 = []


amount2 = []
oriw3 = []
orix3 = []
oriy3 = []
oriz3 = []

amount3 = []
oriw4 = []
orix4 = []
oriy4 = []
oriz4 = []


path = "/home/mads/git/project_in_advanced_robotics/trajectory_learning/traj.csv"

path1= "/home/mads/git/project_in_advanced_robotics/trajectory_learning/demonstrations/tcptrial1.csv"
path2 = "/home/mads/git/project_in_advanced_robotics/trajectory_learning/demonstrations/tcptrial2.csv"
path3 = "/home/mads/git/project_in_advanced_robotics/trajectory_learning/demonstrations/tcptrial3.csv"

step = 0.002
with open(path) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	counter = 0.0
	for row in csv_reader:
		oriw.append(float(row[3]))	
		orix.append(float(row[4]))
		oriy.append(float(row[5]))
		oriz.append(float(row[6]))
		amount.append(counter)
		counter += step


with open(path1) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	counter = 0.0
	for row in csv_reader:
		oriw4.append(float(row[5]))	
		orix4.append(float(row[6]))
		oriy4.append(float(row[7]))
		oriz4.append(float(row[8]))
		amount3.append(counter)
		counter += step


with open(path2) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	counter = 0.0
	for row in csv_reader:
		oriw2.append(float(row[5]))	
		orix2.append(float(row[6]))
		oriy2.append(float(row[7]))
		oriz2.append(float(row[8]))
		amount1.append(counter)
		counter += step


with open(path3) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	counter = 0.0
	for row in csv_reader:
		oriw3.append(float(row[5]))	
		orix3.append(float(row[6]))
		oriy3.append(float(row[7]))
		oriz3.append(float(row[8]))
		amount2.append(counter)
		counter += step

plt.figure(1)	
ax.plot(amount, oriw, color="blue", marker=" ")
ax.plot(amount, orix, color="red", marker=" ")
ax.plot(amount, oriy, color="green", marker=" ")
ax.plot(amount, oriz, color="yellow", marker=" ")

ax.set_xlabel('Time [S]', fontsize=14)
x_patch = mpatches.Patch(color='red', label='X')
y_patch = mpatches.Patch(color='green', label='Y')
z_patch = mpatches.Patch(color='yellow', label='Z')
w_patch = mpatches.Patch(color='blue', label='W')
plt.legend(loc="right", ncol=2, handles=[x_patch, y_patch, z_patch, w_patch])
plt.title("Output quaternions calculated by GMR")


fig2, ax1 = plt.subplots()
plt.figure(2)
ax1.plot(amount1, oriw2, color="blue", marker=" ")
ax1.plot(amount1, orix2, color="red", marker=" ")
ax1.plot(amount1, oriy2, color="green", marker=" ")
ax1.plot(amount1, oriz2, color="yellow", marker=" ")

ax1.plot(amount2, oriw3, color="blue", marker=" ")
ax1.plot(amount2, orix3, color="red", marker=" ")
ax1.plot(amount2, oriy3, color="green", marker=" ")
ax1.plot(amount2, oriz3, color="yellow", marker=" ")

ax1.plot(amount3, oriw4, color="blue", marker=" ")
ax1.plot(amount3, orix4, color="red", marker=" ")
ax1.plot(amount3, oriy4, color="green", marker=" ")
ax1.plot(amount3, oriz4, color="yellow", marker=" ")

ax1.set_xlabel('Time [S]', fontsize=14)
x_patch = mpatches.Patch(color='red', label='X')
y_patch = mpatches.Patch(color='green', label='Y')
z_patch = mpatches.Patch(color='yellow', label='Z')
w_patch = mpatches.Patch(color='blue', label='W')
plt.legend(loc="lower right", ncol=2, handles=[x_patch, y_patch, z_patch, w_patch])
plt.title("Recorded quaternions")

plt.show()