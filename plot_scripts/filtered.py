#!/usr/bin/python
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import csv
import sys
import math

from scipy import signal
from geometry_msgs.msg import WrenchStamped


fs = 500 #sampling frequency

fc = 2.2 # Cut-off frequency of the filter
w = fc / (fs / 2) # Normalize the frequency

b, a = signal.butter(6, w, 'low')
wrench_data = []

fig, ax = plt.subplots()
orig = []
filtered = []
amount = []

	

path = "/home/marcus/project/project_in_advanced_robotics/wrenchwrench1.csv"

with open(path) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	counter = 0
	for row in csv_reader:
		orig.append(float(row[4])*-1.0)
		amount.append(counter)
		counter += 1
		
for i in range(len(orig)):
	wrench_data.append(float(orig[int(i)]))
	if len(wrench_data) > 1000:
		wrench_data.pop(0)	

	f = signal.lfilter(b, a, wrench_data)
	filtered.append(f[len(f)-1])

ax.plot(amount, orig, color="blue", marker=" ")
ax.plot(amount, filtered, color="red", marker=" ")
#ax.plot(amount, forceX, color="red", marker=" ")
ax.set_xlabel('Force Point', fontsize=14)
ax.set_ylabel('Force [N]', fontsize=14)
x_patch = mpatches.Patch(color='red', label='filtered force')
y_patch = mpatches.Patch(color='blue', label='measured force')
plt.legend(loc="upper center", bbox_to_anchor=(0.5, 1.15), ncol=3, handles=[x_patch, y_patch])

plt.show()