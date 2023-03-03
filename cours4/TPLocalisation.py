#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

global idseq 
idseq =0
def getData(data):
	
	global idseq
	print(data)
	if data.header.seq != idseq:
		data.pose.pose.position.x = data.pose.pose.position.x+10
	 	idseq = data.header.seq+1
		data.header.seq = data.header.seq+1
		talker(data)
	
def talker(msg):
    pub.publish(msg)
    
def listener():
	global state,pub
	pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
	rospy.init_node('listener', anonymous=True)
	
	rospy.Subscriber("initialpose", PoseWithCovarianceStamped,getData)
	
	rospy.spin()



if __name__ == '__main__':
	listener()
