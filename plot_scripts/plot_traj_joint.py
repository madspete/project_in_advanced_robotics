#!/usr/bin/python
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import csv
import sys

joint1 = []
joint2 = []
joint3 = []
joint4 = []
joint5 = []
joint6 = []
joint1_1 = []
joint2_1 = []
joint3_1 = []
joint4_1 = []
joint5_1 = []
joint6_1 = []
joint1_2 = []
joint2_2 = []
joint3_2 = []
joint4_2 = []
joint5_2 = []
joint6_2 = []
amount = []

path = "/home/marcus/project/project_in_advanced_robotics/trajectory_learning/traj_joint.csv"
path1 = "/home/marcus/project/project_in_advanced_robotics/traj0.csv"
path2 = "/home/marcus/project/project_in_advanced_robotics/traj2.csv"

with open(path) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	counter = 0
	for row in csv_reader:
		joint1.append(float(row[0]))
		joint2.append(float(row[1]))
		joint3.append(float(row[2]))
		joint4.append(float(row[3]))
		joint5.append(float(row[4]))
		joint6.append(float(row[5]))
		amount.append(counter)
		counter += 1

amount2 = []
with open(path1) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	counter = 0
	for row in csv_reader:
		joint1_1.append(float(row[0]))
		joint2_1.append(float(row[1]))
		joint3_1.append(float(row[2]))
		joint4_1.append(float(row[3]))
		joint5_1.append(float(row[4]))
		joint6_1.append(float(row[5]))
		amount2.append(counter)
		counter += 1

with open(path2) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	counter = 0
	for row in csv_reader:
		joint1_2.append(float(row[0]))
		joint2_2.append(float(row[1]))
		joint3_2.append(float(row[2]))
		joint4_2.append(float(row[3]))
		joint5_2.append(float(row[4]))
		joint6_2.append(float(row[5]))
		counter += 1

plt.figure(1)
#plt.plot(amount, joint1, color="blue", marker=" ")
#plt.plot(amount, joint1_1, color="red", marker=" ")
#plt.plot(amount, joint1_2, color="green", marker=" ")
#plt.figure(2)
#plt.plot(amount, joint2, color="blue", marker=" ")
#plt.plot(amount, joint2_1, color="red", marker=" ")
#plt.plot(amount, joint2_2, color="green", marker=" ")
#plt.figure(3)
plt.plot(amount, joint3, color="blue", marker=" ")
plt.plot(amount2, joint3_1, color="red", marker=" ")
plt.plot(amount2, joint3_2, color="green", marker=" ")
#plt.figure(4)
#plt.plot(amount, joint4, color="blue", marker=" ")
#plt.plot(amount, joint4_1, color="red", marker=" ")
#plt.plot(amount, joint4_2, color="green", marker=" ")
#plt.figure(5)
#plt.plot(amount, joint5, color="blue", marker=" ")
#plt.plot(amount, joint5_1, color="red", marker=" ")
#plt.plot(amount, joint5_2, color="green", marker=" ")
#plt.figure(6)
#plt.plot(amount, joint6, color="blue", marker=" ")
#plt.plot(amount, joint6_1, color="red", marker=" ")
#plt.plot(amount, joint6_2, color="green", marker=" ")

print(len(amount))
print(len(joint6))

plt.show()