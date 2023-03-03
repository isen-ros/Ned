#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
def callback(data):
  print((data))
	#for d in data:
	#	if d==-1:
	#		print(yy)
	#	if d==0:
	#		print(zz)
	#	if d==100:	
	#		print(xx)		          
    
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("map", OccupancyGrid, callback)
   # rospy.Subscriber("map_metadata", MapMetaData, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
     listener()

