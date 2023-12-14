##!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import time, sys, argparse, math
from std_msgs.msg import String, Float32, Float64, Float32MultiArray
import math
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2 
from std_msgs.msg import String
import cv2
import time, sys, argparse, math
import argparse  
import matplotlib.pyplot as plt
#import serial
import os
#import serial.tools.list_ports

import sys, struct
import time
from socket import *
import socket
import os        
import sys
import time
import novatel_oem7_msgs
from novatel_oem7_msgs.msg import BESTPOS
from novatel_oem7_msgs.msg import BESTVEL
from novatel_oem7_msgs.msg import INSPVA
#import keyboard
import cv2
get_lat_flag = 0
obst_det_flag = 0
ros_enable = 0


UDP_IP = "192.168.50.1"
UDP_PORT = 30000

hostName = gethostbyname('192.168.50.2')
portname = 51001

hostName2 = gethostbyname('127.0.0.1')
portname2 = 50000

global mySocket 


SIZE = 1024    # packet size

global steering_center
global steering_limits 
global center_threshold 
global code_start_flag
code_start_flag=1
##turn_circle_radius = 4

# steering_center = -30
##steering_center = -125
steering_center = 180
steering_limits = 400
center_threshold = 25

global acceleration
global brake_1
global brake_2
global required_angle
global duty
global direction
global dist_next_wp
global send
global lat,lng
acceleration=0
brake_1=0
brake_2=0
required_angle=steering_center
duty=0
direction=0
send = [acceleration, brake_1, brake_2, required_angle, duty, direction]




global previous_steering
previous_steering=0 

# 0 for left
# 1 for right





global last_bearing_diff

global steering_angle
global time1,time2

global lat, lng, heading, current_vel, vel_head
current_vel=0
steering_angle=0
heading=0
wp=0


def setup():
    time.sleep(1)
    
    
#if get_lat_flag==0 and ros_enable == 1:
rospy.init_node('record_wp', anonymous=True)

def talker(message):
    pub = rospy.Publisher('logs', String, queue_size=1)
    #rospy.init_node('ni_feedback_node', anonymous=True)
    rospy.loginfo(message)
    pub.publishs(message)

def talker2(message):
    pub = rospy.Publisher('gps', String, queue_size=1)
    #rospy.init_node('ni_feedback_node', anonymous=True)
    rospy.loginfo(message)
    pub.publish(message)

def listener_str(talker_name):
    msg = rospy.wait_for_message(talker_name, String, timeout = 1) ## For one packet
    return msg.data

def listener_s(talker_name):
    msg = rospy.wait_for_message(talker_name, Float32MultiArray, timeout = 1) ## For one packet
    return msg.data

def ros_to_pcl(ros_cloud):
    points_list = []
    
#     for i=0:2:
    for data in pc2.read_points(ros_cloud, skip_nans=True):
#     data = pc2.read_points(ros_cloud, skip_nans=True)
#         print(data)

#         print(points_list)
        points_list.append([data[0], data[1], data[2]])
#         print(points_list)
        
#     pcl_data = pcl.PointCloud_PointXYZRGB()
#     pcl_data.from_list(points_list)

#     return pcl_data
    return points_list


def listener_pcl_y(talker_name):
    msg = rospy.wait_for_message(talker_name, PointCloud2, timeout = 1) ## For one packet
    #curb_pcl = listener_pointcloud("curb_detection_result")
    curb = ((ros_to_pcl(msg)))
    
#     print((len(curb)))
#     print(curb[0][1])
#     print(curb[1][1])
#     return [curb[0][1], curb[11][1]]
    return [curb[0][1], 100.0]



while False:
    curb_pcl = listener_pointcloud("curb_detection_result")
    curb = ((ros_to_pcl(curb_pcl)))
#     print((len(curb[0])))
    print(curb[0][1])
    print(curb[1][1])


global o_flag
o_flag=0

def accelerate(speed1):
    global acceleration
    global brake_1
    global brake_2
    global required_angle
    global duty
    global direction
    global send
    global o_flag
    global time
    global time1
    global time2
    if (speed1>2.0):
        speed1=2.0
        
    acceleration = speed1
    
    if(o_flag==1):
        acceleration=0
    #time1=time.time()
    #print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@time_difference",time1,time2)
    #if (time1-time2>0.4):
    comm_MABX() 


def set_velocity(desired_vel):
    global current_vel
    global acceleration

    desired_vel = float(desired_vel)
    current_vel = float(current_vel)

   # get_vel()
    current_vel = float(current_vel) * 3600 / 1000
    
    print("                                                                                                              ",desired_vel)
    print("                                                                                                              ",current_vel)
    
    
    if desired_vel < current_vel:
        acceleration = acceleration - 0.05
    
        if acceleration < 0:
            acceleration=0

    elif acceleration<1:
    	acceleration=2
    elif desired_vel > current_vel:
        acceleration = acceleration + 0.4
        
#         if acceleration < 1.5:
#             acceleration = 1.5
#         elif acceleration < 2:
#             acceleration = 2
#         elif acceleration < 2.5:
#             acceleration = 2.5
#         else:
#             acceleration = acceleration + 0.5
            
#         if acceleration > (1.75 + desired_vel-current_vel):
#             acceleration = (1.75 + desired_vel-current_vel)
        
    accelerate(acceleration)
    

def apply_brake():
    global acceleration
    global brake_1
    global brake_2
    global required_angle
    global duty
    global direction
    global send
    
    brake_1=0
    brake_2=1
    acceleration = 0
    
    comm_MABX()
    
    time.sleep(1.25)
    
    brake_1=0
    brake_2=0
    
    comm_MABX()
    
    
def remove_brake():
    global acceleration
    global brake_1
    global brake_2
    global required_angle
    global duty
    global direction
    global send
    
    brake_1=1
    brake_2=0
    
    comm_MABX()
    
    time.sleep(1.25)
    
    brake_1=0
    brake_2=0
    
    comm_MABX()

global sol_status, pos_type, lat_delta, lng_delta, sat_used

def callback_latlong(data):
    global lat,lng,heading,current_vel,vel_head 
    global sol_status, pos_type, lat_delta, lng_delta, sat_used
    global latlong_time
    
    latlong_time = time.time()
    
    #rospy.loginfo(data)
    lat = data.lat
    lng = data.lon
    hgt = data.hgt 
    print(lat,lng,hgt)  
    
rospy.Subscriber("/novatel/oem7/bestpos",BESTPOS, callback_latlong)

def callback_vel(data):
    global lat,lng,heading,current_vel,vel_head 
    global sol_status, pos_type, lat_delta, lng_delta, sat_used
    
    #rospy.loginfo(data)
    current_vel=data.hor_speed  
    
rospy.Subscriber("/novatel/oem7/bestvel",BESTVEL, callback_vel)

def callback_heading(data):

    global lat,lng,heading,current_vel,vel_head 
    global sol_status, pos_type, lat_delta, lng_delta, sat_used
    
    #rospy.loginfo(data)
    heading=data.azimuth  
    
rospy.Subscriber("/novatel/oem7/inspva",INSPVA, callback_heading)
global new_file
new_file=time.time()
file=open(str(new_file),"w")
#f.write(f"[")
#lst=[]


while True:
        global lat,lng
        time.sleep(0.15)
        file = open(str(new_file), "a")
        print("done")
        #file.write(str(lat))
        file.writelines("["+str(lat)+","+str(lng)+"],\n")
        file.close()
    
        print(time.time())  
    
        

















i=0
#ports = serial.tools.list_ports.comports(include_links=False)
#head_ser0 = serial.Serial(ports[0].device, 9600)
##head_ser1 = serial.Serial(ports[1].device, 9600)
#head_ser2 = serial.Serial(ports[2].device, 9600)

while 0 and i<100:
    
    head_ser0.flushInput()
    head_ser1.flushInput()
    head_ser2.flushInput()
    i=i+1


def get_distance_metres(aLocation1, aLocation2):

    dlat = aLocation2[0] - aLocation1[0]
    dlong = aLocation2[1] - aLocation1[1]
#   return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.111395e5



    

def get_bearing(aLocation1, aLocation2):

    off_x = aLocation2[1] - aLocation1[1]
    off_y = aLocation2[0] - aLocation1[0]
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;  




if get_lat_flag:
    
    file = open("waypoints.txt", "w")
    #     file.writelines((("time.time(), wp, lat, lng, lat_delta, lng_delta, waypoints[wp][0], waypoints[wp][1], dist_next_wp, Current_Bearing, bearing_ppc, current_bearing_diff_ppc, vel_head, steering, current_vel, acceleration, steer_output, sol_status, pos_type,  sat_used, \n")))
    file.writelines("\n")
    file.close()

i=0

while 0 and i<10:
    
#     set_velocity(13)
    print([float(lat),float(lng)])
##    print(lng)
    get_heading()
    print("heading: ",heading)
    get_vel()
    print("vel mps: ",current_vel)
    print("vel kmph: ",float(current_vel)*3600/1000)
    print("delta: ",[float(lat_delta), float(lng_delta)])
    
    if i > 0:
        print("                                                                       Dist between waypoints:  ", get_distance_metres(prev_pt, [float(lat),float(lng)]))
        print("                                                                       bearing between waypoints:  ", get_bearing(prev_pt, [float(lat),float(lng)]))
        
    
    prev_pt = [float(lat),float(lng)]
    
    print("")
    
    print(time.time())
    
    i = i+1  
    
    time.sleep(0.2)
    
    
    
    
   
    
i=0
lat_a=[]
lng_a=[]

while i<10 and False:
    get_lat()
    lat_a.append(lat)
    lng_a.append(lng)
    i=i+1
##plt.plot(lat_a,lng_a)
##plt.show()
##print((float(max(lat_a)) - float(min(lat_a))) * 1.111395e5)

    
print("heading: ",heading)
while abs(float(heading)-90) > 15 and False:
    print("heading: ",heading)



def set_steering_angle_MABX(desired_angle):
    global required_angle
    required_angle = desired_angle + steering_center
    comm_MABX()
       
  
def fun_bearing_diff(Required_Bearing):
    global heading, Current_Bearing
    
    #get_heading()
    Current_Bearing = heading 
    while Current_Bearing is None:
        get_heading()
        Current_Bearing = heading 
    
    Current_Bearing = float(Current_Bearing)
    bearing_diff = Current_Bearing - Required_Bearing
    #check this logic
    if bearing_diff < -180:
        bearing_diff = (bearing_diff+360)

    if bearing_diff > 180:
        bearing_diff = bearing_diff-360
    #    if abs(bearing_diff)>40:
        
    return bearing_diff


def return_to_zero_MABX():
    global acceleration
    global brake_1
    global brake_2
    global required_angle
    global duty
    global direction
    global send
    
    set_steering_angle_MABX(0)
    
    
while 0:
#     return_to_zero_MABX()
    set_steering_angle_MABX(0)
    print(read_angle())


def get_distance_metres(aLocation1, aLocation2):

    dlat = aLocation2[0] - aLocation1[0]
    dlong = aLocation2[1] - aLocation1[1]
#   return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.111395e5

  
    
current_bearing_diff = 0
##while True:
##    print(read_angle())
##    time.sleep(0.5)

print("Started:")
setup()
 


import math
def get_location_metres(aLocation1, dNorth, dEast):
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*aLocation1[0]/180))

    #New position in decimal degrees
    newlat = aLocation1[0] + (dLat * 180/math.pi)
    newlon = aLocation1[1] + (dLon * 180/math.pi)
    
    return [newlat, newlon]



#time2=time.time()
waypoints_TB = [
##[17.60198679053,78.12683546152],  # HET
##[17.60202062436,78.12710034589], # corner
##[17.60201073128, 78.12703023101], # T L2/2 HEW L2/2
[17.60202642385712, 78.12709051289234], # T L2/2 HEW L2/2
[17.60232107934, 78.12710417701], # BS L2
##[17.602495132134248, 78.1271199642713], # Cement Road E L1
[17.60250411528709, 78.1271199642713], # Cement Road E L1      1 0 
[17.60249748494463, 78.12689710905947], # CR Mid L1
##[17.60260917313, 78.12712994452], # SC E L1
##[17.60271794445, 78.12702341396], # SC NE L1
##[17.60275621608, 78.12687007749], # SC N L1
##[17.60275621873, 78.12686989372], # SC NW L1
##[17.60264044892, 78.12662998655], # SC W L1
##[17.6024729962359, 78.12661980466379], # Sement road L1/3
[17.60249056886478, 78.12662880271957], # Sement road L1/3
[17.60248817438, 78.12662407411], # BS2 N L1/3
##[17.60248799817, 78.12662414236], # BS2 L0/3
##[17.60213361772, 78.12666241622], # BS2 S L0/3
[17.60206553637, 78.12663011149], # BS2 S L1/3 VMS
##[17.60198707307, 78.12663361179], # T L1/3 HEWL1/2
[17.60199605622284, 78.12663361179], # T L1/3 HEWL1/2     1 0
[17.60198679053,78.12683546152]     # HET
]


waypoints = []

# waypoints.extend(waypoints1)
# waypoints.extend(waypoints2)


# waypoints.extend(waypoints12)
# waypoints.extend(waypoints3)
# waypoints.extend(waypoints4)
# waypoints.extend(waypoints5)
# waypoints.extend(waypoints6)

# waypoints = waypoints_reverse2

waypoints = waypoints_TB


wp = 0
#wp = 288
# wp = 34

i=0
dist_flag=1


print(len(waypoints))

while i < (len(waypoints)-1):
    if(get_distance_metres(waypoints[i],waypoints[i+1])  > 20):
        print("############################################################## waypoint intwrpolated")
        new_pt = [(waypoints[i][0]+waypoints[i+1][0])/2, (waypoints[i][1]+waypoints[i+1][1])/2]
        waypoints.insert(i+1, new_pt)
    elif(get_distance_metres(waypoints[i],waypoints[i+1])  < 5):
        diff = abs((abs(get_bearing(waypoints[i],waypoints[i-1]))) - (abs(get_bearing(waypoints[i+1],waypoints[i]))))
        if (i > 1) and (diff > 10 and diff < 350):
            print("diff                               ",diff)
            i = i+1
        else:
            print("############################################################## waypoint Removed")
#        new_pt = [(waypoints[i][0]+waypoints[i+1][0])/2, (waypoints[i][1]+waypoints[i+1][1])/2]
            del waypoints[i+1]
#         waypoints.insert(i+1, new_pt)
    else:
        print(get_distance_metres(waypoints[i],waypoints[i+1]))
        i=i+1

print(len(waypoints))

# i=0
# while i < (len(waypoints)-1):
#     if(get_distance_metres(waypoints[i],waypoints[i+1])  > 20):
#         print("############################################################## waypoint intwrpolated")
#         new_pt = [(waypoints[i][0]+waypoints[i+1][0])/2, (waypoints[i][1]+waypoints[i+1][1])/2]
#         waypoints.insert(i+1, new_pt)
#     elif(get_distance_metres(waypoints[i],waypoints[i+1])  < 7):
#         print("############################################################## waypoint Removed")
# #        new_pt = [(waypoints[i][0]+waypoints[i+1][0])/2, (waypoints[i][1]+waypoints[i+1][1])/2]
#         del waypoints[i+1]
# #         waypoints.insert(i+1, new_pt)
#     else:
#         print(get_distance_metres(waypoints[i],waypoints[i+1]))
#         print("                                    ", get_bearing(waypoints[i],waypoints[i+1]))
#         i=i+1

# print(len(waypoints))


i=0
dist = 1000
 
while i < (len(waypoints)-1):
    if((get_distance_metres(waypoints[i],[float(lat), float(lng)]) < dist) ):
        dist = (get_distance_metres(waypoints[i],[float(lat), float(lng)]))
        print(dist)
        print(abs(get_bearing(waypoints[i],[float(lat), float(lng)])))
        i = i+1
    else:
        if (abs(get_bearing(waypoints[i],[float(lat), float(lng)])) < 10 ) or (abs(get_bearing(waypoints[i],[float(lat), float(lng)])) > 350):
            dist = (get_distance_metres(waypoints[i],[float(lat), float(lng)]))
            print(dist)
            print(dist)
            print(dist)
            print(abs(get_bearing(waypoints[i],[float(lat), float(lng)])))
            print(abs(get_bearing(waypoints[i],[float(lat), float(lng)])))
            print(abs(get_bearing(waypoints[i],[float(lat), float(lng)])))
            break
        else:
            dist = (get_distance_metres(waypoints[i-2],[float(lat), float(lng)]))
            i = i+1
    
    

##print("length: ",len(waypoints))
print(i)

# wp = i

##while True:
##    time.sleep(1)
   

# wp = 0

i = 0
pos = [0.0, 0.0]
while(0):
##while(True):
    get_lat()
    currentLocation = [float(lat), float(lng), 0.0]
    while currentLocation is None:
        get_lat()
        currentLocation = [float(lat), float(lng), 0.0]
        
    position = [float(lat), float(lng)]
    print("position = ", position)
    i = i+1
    pos = [ pos[0] + position[0], pos[1] + position[1] ]
##    print("position = ", pos)
    pos1 = [pos[0] / i , pos[1]/i ]
    print("position = ", pos1)
    print("error = ", (np.linalg.norm(np.array(position)- pos1) * 1.111395e5)        )
    
    
     
        
set_velocity(0)         
##time.sleep(5)

while False:
    set_velocity(3)
    time.sleep(0.1)
#     set_steering_angle_MABX(200)
#     read_angle()
    time.sleep(0.2)
    
    
    
  #  time.sleep(0.5)
# apply_brake()
# remove_brake()




##write(0)
accelerate(0)
##return_to_zero()
##time.sleep(5)
#get_heading()
Current_Bearing = heading
#Current_Bearing = listener("vel_head")
while Current_Bearing is None:
    get_heading()
    Current_Bearing = heading
    #Current_Bearing = get_heading()
    #               Current_Bearing = listener("vel_head")
Current_Bearing = float(Current_Bearing)
print("Current_Bearing = ",Current_Bearing)

##file_name = "log/" + str(time.gmtime().tm_mday)+"_"+str(time.gmtime().tm_mon)+"_"+str(time.gmtime().tm_year)+"_"+str(time.gmtime().tm_hour)+":"+str(time.gmtime().tm_min)+".txt"
file_name = "log/" + time.ctime(time.time())[11:19] + "_" + time.ctime(time.time())[8:10] + "" + time.ctime(time.time())[4:7] + time.ctime(time.time())[20:24] + ".txt"
print(file_name)
##file = open("log/" + str(time.gmtime().tm_mday)+"_"+str(time.gmtime().tm_mon)+"_"+str(time.gmtime().tm_year)+"_"+str(time.gmtime().tm_hour)+":"+str(time.gmtime().tm_min)+".txt", "w")
# file = open(file_name, "w")
# file.writelines((("time.time(), wp, lat, lng, lat_delta, lng_delta, waypoints[wp][0], waypoints[wp][1], dist_next_wp, Current_Bearing, bearing_ppc, current_bearing_diff_ppc, vel_head, steering, current_vel, acceleration, steer_output, sol_status, pos_type,  sat_used, \n")))
# file.close()
#while True:
#    talker('str(time.time())+", " + str(wp)+", " + str( lat)+", " + str( lng)+", " + str( lat_delta)+", " + str( lng_delta)+", " + str( waypoints[wp][0])+", " + str( waypoints[wp][1])+", " + str( dist_next_wp)+", " + str( Current_Bearing)+", " + str( bearing_ppc)+", " + str( current_bearing_diff_ppc)+", " +str( vel_head) + "," + str( steering)+", " + str( current_vel)+", " + str( acceleration)+", " + str( steer_output)+", " + str( sol_status)+", " + str( pos_type)+", " + str(  sat_used)')
#    time.sleep(0.5)
bus1 = 1

current_bearing_diff_ppc = 0
Ld_steer = 8

steer_output = 0
curb_dist=[4, 4]

curb_count = 0

obsdist = 200

if obst_det_flag:
    #proc_od = subprocess.Popen([sys.executable, '/home/suzuki/Desktop/Akshay/25062022/yolo/yolov5/yolov5_distance_mabx_ros.py'])
    print("Started obstacle detection process:")
    #print(proc_od.pid)
    time.sleep(2)
    

traffic_flag = 1
acceleration = 1.5
while not rospy.is_shutdown():
##while True:

    try:
        
 #       obsdist = obs_dist()
#        get_lat()
        print(lat,lng)
        currentLocation = [float(lat), float(lng), 0.0]
        while currentLocation is None:
        	#get_lat()
        	currentLocation = [float(lat), float(lng), 0.0]
        position = [float(lat), float(lng)]
        dist_next_wp = (np.linalg.norm(np.array(position)- waypoints[wp]) * 1.111395e5)    
        
        curb_dist = get_curb_dist()
        
#        print("Total curb-to-curb length : ", curb_dist[0] - curb_dist[1])
        print("Vehicle Location from Left Curb : ", curb_dist[0])
#        print("Vehicle Location from Right Curb : ", curb_dist[1])
     #   print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$",desired_vel) 
        
        if(curb_dist[0] < 1.5 and curb_dist[0] > 0):
            
            steer_output = 50 + (750 * np.arctan( -1 * 2 * 3.5 * np.sin(np.pi * current_bearing_diff_ppc / 180) / Ld_steer  ))
            accelerate(1.5)
            set_steering_angle_MABX(steer_output)
            curb_count = curb_count + 1
            print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  $$$$$$$$$$$$  CURB CORRECTION !!!!!!!!! ", curb_count)
                    
#         elif(curb_dist[0] < 3):
# 
#             steer_output = -50 + (750 * np.arctan( -1 * 2 * 3.5 * np.sin(np.pi * current_bearing_diff_ppc / 180) / Ld_steer  ))
#             accelerate(1.5)
#             set_steering_angle_MABX(steer_output)
            
        else:
            
            set_steering_angle_MABX(steer_output)
            
            
            
        if(obsdist < 20.0):
            accelerate(0)
            o_flag = 1
            print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ obstacle")
            obsdist = obs_dist()
        else:
            o_flag=0


      
        
        
        
        
#         get_vel()
        
        print("vel in kmph = ",float(current_vel))
        print("wp =============== ",wp)
        print(time.time())
        
      #  get_lat()
        currentLocation = [float(lat), float(lng), 0.0]
        while currentLocation is None:
            #get_lat()
            currentLocation = [float(lat), float(lng), 0.0]
            
        position = [float(lat), float(lng)]
    
        
#         get_heading()
        
        
        
        
        
    #print("position = ", position)
        if ((np.linalg.norm(np.array(position)- waypoints[len(waypoints)-1]) * 1.111395e5) > 7) or (wp<5):
            
            #print("distance to last waypoint", (np.linalg.norm(np.array(position)- waypoints[len(waypoints)-1]) * 1.111395e5) )
            if (wp == len(waypoints)):
                print("FINISHED at point 1")
                accelerate(0)
                return_to_zero()
                apply_brake()
                time.sleep(2)
                remove_brake()
                break
                dist_next_wp = 0.0001
                off_y = 0.0001 #- currentLocation[0] + waypoints[wp][0]
                off_x = 0.0001 #- currentLocation[1] + waypoints[wp][1]
            
            else:
                obsdist = obs_dist()
                
                curb_dist = get_curb_dist()
        
#                print("Total curb-to-curb length : ", curb_dist[0] - curb_dist[1])
                print("Vehicle Location from Left Curb : ", curb_dist[0])
#                print("Vehicle Location from Right Curb : ", curb_dist[1])
                
                
                if(curb_dist[0] < 1.5  and curb_dist[0] > 0):
                    
                    steer_output = 50 + (750 * np.arctan( -1 * 2 * 3.5 * np.sin(np.pi * current_bearing_diff_ppc / 180) / Ld_steer  ))
                    accelerate(1.5)
                    set_steering_angle_MABX(steer_output)
                    curb_count = curb_count + 1
                    print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  $$$$$$$$$$$$  CURB CORRECTION !!!!!!!!! ", curb_count)
        #         elif(curb_dist[0] < 3):
        # 
        #             steer_output = -50 + (750 * np.arctan( -1 * 2 * 3.5 * np.sin(np.pi * current_bearing_diff_ppc / 180) / Ld_steer  ))
        #             accelerate(1.5)
        #             set_steering_angle_MABX(steer_output)
                    
                else:
                    
                    set_steering_angle_MABX(steer_output)
                    
                    
                if(obsdist < 20.0):
                    accelerate(0)
                    o_flag=1
                    obsdist = obs_dist()
                    print("                                                 obstacle detected !!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                else:
                    o_flag=0
                    
                    
#             get_lat()
#             
#             currentLocation = [float(lat), float(lng), 0.0]
#             while currentLocation is None:
#                 get_lat()
#                 currentLocation = [float(lat), float(lng), 0.0]
#             position = [float(lat), float(lng)]
            
            if False and (bus1 == 1) and ((np.linalg.norm(np.array(position)- [17.60232107934, 78.12710417701]) * 1.111395e5) < 5):
                bus1 = 0
                accelerate(0) 
                time.sleep(5)
                
                #if obst_det_flag:
                #    proc_od.kill()
                time.sleep(1)
                
                #proc = subprocess.Popen([sys.executable, '/home/suzuki/DEMO/ros_trafficlight.py'])
                print("Started process:")
                #print(proc.pid)

                label = listener_str('label')
                print(label)
                print(label)
                print(label)
                print(label)
                print(label)
                print(label)
                print(label)
                print(label)
                print(label)
                print(label)
                print(label)
                print(label)
                print(label)
                print(label)
                
                while not (label=='go' or label=='goLeft' or label == 'goRight'):
#                 while (label=='stop' or label=='warning'):
                    accelerate(0) 
                    label = listener_str('label')
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
            
                
                #proc.kill()
                print("Killing traffic process:")
                
                if obst_det_flag:
                    #proc_od = subprocess.Popen([sys.executable, '/home/suzuki/Desktop/Akshay/25062022/yolo/yolov5/yolov5_distance_mabx_ros.py'])
                    print("Started obstacle detection process:")
                    #print(proc_od.pid)


    #             time.sleep(10)
                
         
                    
                    
        
     
#    Traffic Light actualll
    
            if  (wp==0 and traffic_flag == 0 and dist_next_wp<18) :
            
            #if 0:
                traffic_flag=0
                accelerate(0)
                
                print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ TRAFFIC")
                #if obst_det_flag:
                #    proc_od.kill()
                #time.sleep(2)
                print("KILL obstacle Detection process")
                print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ TRAFFIC")
                print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ TRAFFIC")
                
                
                #proc = subprocess.Popen([sys.executable, '/home/vision/Downloads/pypylon/ros_trafficlight.py'])
                #print("Started traffic process:")
                #print(proc.pid)
                print(time.time())
                #time.sleep(15)
                print(time.time())

                label = listener_str('label')
                
                print(label)
                print("traffic_flag : ",traffic_flag)
#                 while label is None:
#                     label = listener('label')
                print(time.time())
                
                while not (label=='go' or label=='goLeft' or label == 'goRight'):
#                 while (label=='stop' or label=='warning'):
                    accelerate(0) 
                    label = listener_str('label')
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
#                     while label is None:
#                         label = listener('label')
#                         print(label)
                        
                
                
                print(time.time())
                #proc.kill()
                #print("Killing traffic process:")
                
                if obst_det_flag:
                    #proc_od = subprocess.Popen([sys.executable, '/home/suzuki/Desktop/Akshay/25062022/yolo/yolov5/yolov5_distance_mabx_ros.py'])
                    print("Starting obstacle detection process:")
                    #print(proc_od.pid)



    #             time.sleep(10)
                #get_lat()
                currentLocation = [float(lat), float(lng), 0.0]
                while currentLocation is None:
                    get_lat()
                    currentLocation = [float(lat), float(lng), 0.0]
                position = [float(lat), float(lng)]
                        
                
                
                
                
                
                #################################
            traffic_flag=1        
            if  (wp==1 and traffic_flag == 0 and dist_next_wp<12) :

            #if 0:
                traffic_flag=0
                accelerate(0)

                print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                #if obst_det_flag:
                #    proc_od.kill()
                #time.sleep(2)
                print("KILL obstacle Detection process")
                print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")


                #proc = subprocess.Popen([sys.executable, '/home/vision/Downloads/pypylon/ros_traffi>
                #print("Started traffic process:")
                #print(proc.pid)
                print(time.time())
                #time.sleep(15)
                print(time.time())

                label = listener_str('label')

                print(label)
                print("traffic_flag : ",traffic_flag)
#                 while label is None:
#                     label = listener('label')
                print(time.time())

                while not (label=='go' or label=='goLeft' or label == 'goRight'):
#                 while (label=='stop' or label=='warning'):
                    accelerate(0) 
                    label = listener_str('label')
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
                    print(label)
#                     while label is None:
#                         label = listener('label')
#                         print(label)

               

                print(time.time())

            dist_next_wp = (np.linalg.norm(np.array(position)- waypoints[wp]) * 1.111395e5)
            off_y = - currentLocation[0] + waypoints[wp][0]
            off_x = - currentLocation[1] + waypoints[wp][1]
            waypointLoc = waypoints[wp]
            print("waypoint",waypoints[wp])
            
            #print("position = ", position)
            bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
            if bearing < 0:
                bearing += 360.00
        
#             print("required bearing = ",bearing)
            
            

            bearing_threshold = 180 * np.arctan(3.5 / dist_next_wp) / (3.4 * 3)
##            print("distance to next waypoint", dist_next_wp )
            if(bearing_threshold<2):
                bearing_threshold=2
            if(bearing_threshold>5):
                bearing_threshold=5
            
##            print("bearing_threshold = ",bearing_threshold)
            steering_angle = read_angle()
            
##            dashboard(currentLocation, wp, waypoints[wp], dist_next_wp, bearing, heading, steering)
            
            required_steer_rate = 25
            duty=required_steer_rate
#             set_velocity(12)
            
            

            if wp>0:
                dist_prev_wp = (np.linalg.norm(np.array(position)- waypoints[wp-1]) * 1.111395e5)
##                print("dist_prev_wp = ",dist_prev_wp)
         

            Ld = 6 #8
            
            
            bearing_ppc = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
            if bearing_ppc < 0:
                bearing_ppc += 360.00
        
            print("required bearing ppc = ",bearing_ppc)
            
#             get_heading()
#             Current_Bearing = heading
#             print("Current_Bearing = ",Current_Bearing)

            current_bearing_diff_ppc = fun_bearing_diff(bearing_ppc)
            print("                                                                                                                          current_bearing_diff_ppc= ", current_bearing_diff_ppc)
            
            Ld_steer = 8
            
            if(abs(current_bearing_diff_ppc) > 35):
                set_velocity(10)
#                 print("")
#                 Ld_steer = 4
            else:
                set_velocity(10)
#                 Ld_steer = 8
               
#             steer_output = 700 * np.arctan( -1 * 2 * 3.5 * np.sin(np.pi * current_bearing_diff_ppc / 180) / Ld_steer  )
            steer_output = 750 * np.arctan( -1 * 2 * 3.5 * np.sin(np.pi * current_bearing_diff_ppc / 180) / Ld_steer  )
            
##            if
#             print("                                              required steer_output = ",steer_output)
            print("                                              required steer angle = ",steer_output + steering_center)
            #steering = read_angle()
            print("                                              Current steering angle = ",steering_angle)
            print("                                                                      steering angle difference = ", (steer_output + steering_center) - steering_angle)
            
            set_steering_angle_MABX(steer_output)
            print("dist_next_wp = ",dist_next_wp)
                #indicat                                                 or="0"

            
##            log_a.append([time.time(), wp, lat, lng, lat_delta, lng_delta, waypoints[wp][0], waypoints[wp][1], dist_next_wp, Current_Bearing, bearing_ppc, current_bearing_diff_ppc, steering, current_vel, acceleration, steer_output, sol_status, pos_type,  sat_used])
##            file = open("log/" + str(time.gmtime().tm_mday)+"_"+str(time.gmtime().tm_mon)+"_"+str(time.gmtime().tm_year)+"_"+str(time.gmtime().tm_hour)+":"+str(time.gmtime().tm_min)+".txt", "a")


#             file = open(file_name, "a")
#             file.writelines(str(time.time())+", " + str(wp)+", " + str( lat)+", " + str( lng)+", " + str( lat_delta)+", " + str( lng_delta)+", " + str( waypoints[wp][0])+", " + str( waypoints[wp][1])+", " + str( dist_next_wp)+", " + str( Current_Bearing)+", " + str( bearing_ppc)+", " + str( current_bearing_diff_ppc)+", " +str( vel_head) + ", " + str( steering)+", " + str( current_vel)+", " + str( acceleration)+", " + str( steer_output)+", " + str( sol_status)+", " + str( pos_type)+", " + str(  sat_used) + ", \n")
#             file.close()
 
            
            
##            talker(str(time.time())+", " + str(wp)+", " + str( lat)+", " + str( lng)+", " + str( lat_delta)+", " + str( lng_delta)+", " + str( waypoints[wp][0])+", " + str( waypoints[wp][1])+", " + str( dist_next_wp)+", " + str( Current_Bearing)+", " + str( bearing_ppc)+", " + str( current_bearing_diff_ppc)+", " +str( vel_head) + ", " + str( steering)+", " + str( current_vel)+", " + str( acceleration)+", " + str( steer_output)+", " + str( sol_status)+", " + str( pos_type)+", " + str(  sat_used))
##            talker2( str( lat)+", " + str( lng) + ", " + str( Current_Bearing))
            
            
            time.sleep(0.3)
            print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@loop Finished")
            time2=time.time()
            if (wp < len(waypoints)):
##                print("wp : ",wp)
##                print("((np.linalg.norm(np.array(position)- waypoints[wp]) * 1.111395e5) ) : ",((np.linalg.norm(np.array(position)- waypoints[wp]) * 1.111395e5) ))
                if (((np.linalg.norm(np.array(position)- waypoints[wp]) * 1.111395e5) < Ld ) and (wp<len(waypoints))):
                    print("##################################33 WP Changed")
                    wp=wp+1
                
        else:
#       if ((np.linalg.norm(np.array(position)- waypoints[len(waypoints)-1]) * 1.111395e5) < 5) and wp>3:
            print("FINISHED")
            
#            if obst_det_flag:
#                    proc_od.kill()
                    
            accelerate(0)
            #return_to_zero()
            print("braking-c")
            apply_brake()
            time.sleep(1)
            remove_brake()
            time.sleep(1)
            break
        
            




    #except rospy.ROSInterruptException:
    except False:
        pass



