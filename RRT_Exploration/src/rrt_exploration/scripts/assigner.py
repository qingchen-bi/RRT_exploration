#!/usr/bin/env python

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf
from rrt_exploration.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot,informationGain,discount
from numpy.linalg import norm
#---------------------------------------
# from std_msgs.msg import Int8
#--------------------------------------
# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
global1=OccupancyGrid()
global2=OccupancyGrid()
global3=OccupancyGrid()
globalmaps=[]

#----------------------------------------------
# flag_pk = Int8()
#----------------------------------------------
def callBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))
	
def mapCallBack(data):
    global mapData
    mapData=data
# Node----------------------------------------------

def node():
	global frontiers,mapData,global1,global2,global3,globalmaps
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/map')
	info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier=rospy.get_param('~info_multiplier',3.0)		# This parameter is used to give importance to information gain of a frontier point over the cost         lanbuda 
	hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range,,,,      hrad
	hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region,,,,,,,,     hgain
	frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')	# come from filter
	n_robots = rospy.get_param('~n_robots',1)
	namespace = rospy.get_param('~namespace','')
	namespace_init_count = rospy.get_param('namespace_init_count',1)
	delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)
	rateHz = rospy.get_param('~rate',100)
	
	rate = rospy.Rate(rateHz)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack) # save map data 
	rospy.Subscriber(frontiers_topic, PointArray, callBack) # save filtered frontier points
#---------------------------------------------------------------------------------------------------------------
#---------------------------------------------------------------------------------------------------------------
	# pk_pub = rospy.Publisher('/pk',Int8,queue_size = 10)
#---------------------------------------------------------------------------
# wait if no frontier is received yet 
	while len(frontiers)<1:
		pass
	centroids=copy(frontiers)	
#wait if map is not received yet
	while (len(mapData.data)<1):
		pass

	robots=[] # robot class
	if len(namespace)>0:
		for i in range(0,n_robots):
			robots.append(robot(namespace+str(i+namespace_init_count))) # robot group  # robot class
	elif len(namespace)==0:
			robots.append(robot(namespace))
	for i in range(0,n_robots):
		robots[i].sendGoal(robots[i].getPosition()) # robot's goal
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():
		centroids=copy(frontiers)		
#-------------------------------------------------------------------------			
#Get information gain for each frontier point
		infoGain=[]
		for ip in range(0,len(centroids)):
			infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)) # caculation I  and save
#-------------------------------------------------------------------------			
#get number of available/busy robots
		na=[] #available robots
		nb=[] #busy robots
		for i in range(0,n_robots):
			if (robots[i].getState()==1): # busy is == 1
				nb.append(i)
			else:
				na.append(i)	
		rospy.loginfo("available robots: "+str(na))	# zhi ming shi na yi ge 
#------------------------------------------------------------------------- 
#get dicount and update informationGain
		for i in nb+na:
			infoGain=discount(mapData,robots[i].assigned_point,centroids,infoGain,info_radius)
#-------------------------------------------------------------------------            
		revenue_record=[]
		centroid_record=[]
		id_record=[]
		
		for ir in na: # caculation every na robot and every centroids 's R
			for ip in range(0,len(centroids)):
				cost=norm(robots[ir].getPosition()-centroids[ip])	# ir robot  and ip frontier 's distance (cost N)
				threshold=1
				information_gain=infoGain[ip] #  I 
				if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):

					information_gain*=hysteresis_gain
				revenue=information_gain*info_multiplier-cost # R
				revenue_record.append(revenue) # the R save oder(index) == ir
				centroid_record.append(centroids[ip])
				id_record.append(ir)
		
		if len(na)<1: # no na robot caculation nb robot and every centroids 's R
			revenue_record=[]
			centroid_record=[]
			id_record=[]
			for ir in nb:
				for ip in range(0,len(centroids)):
					cost=norm(robots[ir].getPosition()-centroids[ip])		
					threshold=1
					information_gain=infoGain[ip]
					if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
						information_gain*=hysteresis_gain
				
					if ((norm(centroids[ip]-robots[ir].assigned_point))<hysteresis_radius): # caculation every nb robot's assigned point  and every centroids distance
						information_gain=informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*hysteresis_gain #  if in near update infomation gain

					revenue=information_gain*info_multiplier-cost
					revenue_record.append(revenue) # the R save oder(index) == ir
					centroid_record.append(centroids[ip])
					id_record.append(ir)
		
		rospy.loginfo("revenue record: "+str(revenue_record))	
		rospy.loginfo("centroid record: "+str(centroid_record))	
		rospy.loginfo("robot IDs record: "+str(id_record))	
		
#-------------------------------------------------------------------------	
		if (len(id_record)>0):
			winner_id=revenue_record.index(max(revenue_record)) # the R save oder(index) == ir
			
			robots[id_record[winner_id]].sendGoal(centroid_record[winner_id]) # the R save oder(index) == ir
#----------------------------------------------------------------------------------------
			# if(flag_pk.data == 0):
			# 	flag_pk.data = 1
			# 	pk_pub.publish(flag_pk)
#----------------------------------------------------------------------------------------
	
			rospy.loginfo(namespace+str(namespace_init_count+id_record[winner_id])+"  assigned to  "+str(centroid_record[winner_id]))	
			rospy.sleep(delay_after_assignement)
#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
