#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from nav_msgs.msg import OccupancyGrid
import numpy as np

state = 0
OriginX =0
OriginY = 0
w= 0
h =0
reso = 0
map1D = []
def Tab1DtoTab2D(tab1D,width,height):
	#Convertit le tableau en 1D vers du 2D 
	res = np.reshape(tab1D,(width, height))
	#print(res)
	return (res)
	

def Tab2DtoTab1D(tab2D):
	
	res = np.reshape(tab2D,-1)
	return (res)

def IJtoXY(i,j,resolution,oX = 0,oY = 0):

	x=i*resolution + oX 
	y=j*resolution + oY

	return (x,y)

def XYtoIJ(x,y,resolution,oX = 0,oY = 0):
	
	i = int(round((x-oX)/resolution))
	j = int(round((y-oY)/resolution))

	return (i,j)




def getData(data):
	#print(type(data))
	global OriginX,OriginY,w,h,reso

	OriginX = data.info.origin.position.x # C'est la postion orginal 
	OriginY = data.info.origin.position.y # de la map en -10;-10
	w = data.info.width
	h = data.info.height
	reso = data.info.resolution
	global map1D 
	map1D = data.data
	map2D = Tab1DtoTab2D(map1D,w,h)
       	print(map2D)
	print(IJtoXY(20,20,reso,OriginX,OriginY))
	cr  = XYtoIJ(0.2,0.04,reso,OriginX,OriginY)
	print(cr)
	print(map2D[cr[0]][cr[1]])
	exit(0)
#    rospy.loginfo("OccupancyGrid x : %d", data.origin.position.x)
#    rospy.loginfo("OriX : %d", data.geometry_msgs.Pose.origin)
#    rospy.loginfo("pos x:  %d", data.origin.position.x)
#    rospy.loginfo("pos y:  %d", data.origin.position.y)
#    rospy.loginfo("pos z:  %d", data.origin.position.z)
#    rospy.loginfo("width: %d", data.width)
#    rospy.loginfo("height: %d", data.height)
#    rospy.loginfo("reso: %f",data.resolution)

   
    
def listener():
	global state
	rospy.init_node('listener', anonymous=True)
	
	rospy.Subscriber("map", OccupancyGrid,getData)
	# spin() simply keeps python from exiting until this node is stopped
	
	rospy.spin()

if __name__ == '__main__':
	listener()
	#test =[1,2,3,4,5,6,7,8,9]
	#print(Tab1DtoTab2D(test,3,3))
	#print(Tab2DtoTab1D(Tab1DtoTab2D(test,3,3)))
	#res = IJtoXY(5,2,0.2,3,0)
	#print(res)
	#res2 = XYtoIJ(res[0],res[1],0.2,3,0)
	#print(res2)
	
