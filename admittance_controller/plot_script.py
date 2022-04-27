import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import csv



path = "/home/mads/git/project_in_advanced_robotics/admittance_controller/joints.csv"
joint1 = []
joint2 = []
joint3 = []
joint4 = []
joint5 = []
joint6 = []
amount = []

step = 0.002
with open(path) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	counter = 0.0
	for row in csv_reader:
		joint1.append(float(row[0]))	
		joint2.append(float(row[1]))	
		joint3.append(float(row[2]))	
		joint4.append(float(row[3]))	
		joint5.append(float(row[4]))	
		joint6.append(float(row[5]))	
		amount.append(counter)
		counter += step


plt.plot(amount, joint1)
plt.plot(amount, joint2)
plt.plot(amount,joint3)
plt.plot(amount, joint4)
plt.plot(amount, joint5)
plt.plot(amount, joint6)
plt.show()