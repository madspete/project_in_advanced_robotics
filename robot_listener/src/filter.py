from re import I
from scipy import signal
import matplotlib.pyplot as plt
import csv
import rospy
from geometry_msgs.msg import WrenchStamped


path = "/home/mads/Downloads/wrenchwrench1.csv"
wrenchx = []
wrenchy = []
wrenchz = []
wrenchtx = []
wrenchty = []
wrenchtz = []
with open(path) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	counter = 0.0
	for row in csv_reader:
		wrenchx.append(float(row[2]))	
		wrenchy.append(float(row[3]))	
		wrenchz.append(float(row[4]))	
		wrenchtx.append(float(row[5]))	
		wrenchty.append(float(row[6]))	
		wrenchtz.append(float(row[7]))	

fs = 500 #sampling frequency

fc = 2.2 # Cut-off frequency of the filter
w = fc / (fs / 2) # Normalize the frequency

b, a = signal.butter(6, w, 'low')
wrench_datax = []
wrench_datay = []
wrench_dataz = []
wrench_datatx = []
wrench_dataty = []
wrench_datatz = []


pub = rospy.Publisher('/filtered_wrench', WrenchStamped, queue_size=10)
index = 0
max_index = len(wrenchz) - 1 
def callback(data):
	global index, b, a, max_index
	if index <= max_index:
		fx = wrenchx[index]
		fy = wrenchy[index]
		fz = wrenchz[index]
		tx = wrenchtx[index]
		ty = wrenchty[index]
		tz = wrenchtz[index]
		wrench_datax.append(fx)
		wrench_datay.append(fy)
		wrench_dataz.append(fz)
		wrench_datatx.append(tx)
		wrench_dataty.append(ty)
		wrench_datatz.append(tz)
		if len(wrench_datax) > 1000:
			wrench_datax.pop(0)
			wrench_datay.pop(0)
			wrench_dataz.pop(0)
			wrench_datatx.pop(0)
			wrench_dataty.pop(0)
			wrench_datatz.pop(0)

		filtered = signal.lfilter(b, a, wrench_datax)
		data.wrench.force.x = filtered[len(filtered)-1]
		filtered = signal.lfilter(b, a, wrench_datay)
		data.wrench.force.y = filtered[len(filtered)-1]
		filtered = signal.lfilter(b, a, wrench_dataz)
		data.wrench.force.z = filtered[len(filtered)-1]
		filtered = signal.lfilter(b, a, wrench_datatx)
		data.wrench.torque.x = filtered[len(filtered)-1]
		filtered = signal.lfilter(b, a, wrench_dataty)
		data.wrench.torque.y = filtered[len(filtered)-1]
		filtered = signal.lfilter(b, a, wrench_datatz)
		data.wrench.torque.z = filtered[len(filtered)-1]

	pub.publish(data)


def listener():
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("/wrench", WrenchStamped, callback)

	rospy.spin()


if __name__ == '__main__':
	listener()