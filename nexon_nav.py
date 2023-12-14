import socket
import time
import tcp
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from novatel_oem7_msgs.msg import BESTPOS
from novatel_oem7_msgs.msg import BESTVEL
from novatel_oem7_msgs.msg import INSPVA

# global aeb_flag


global lat, lng, heading, current_vel, vel_head, aeb_flag
aeb_flag = 2


def callback_latlong(data):
    global lat,lng,heading,current_vel,vel_head 
    lat = data.lat
    lng = data.lon

def callback_heading(data):

    global lat,lng,heading,current_vel,vel_head 
    #rospy.loginfo(data)
    heading=data.azimuth  

def callback_aeb(msg):
	global aeb_flag 
	aeb_flag = msg.data


rospy.init_node('nav_algo', anonymous=True)

rospy.Subscriber("/radar_ros_aeb",Float32, callback_aeb)
rospy.Subscriber("/novatel/oem7/bestpos",BESTPOS, callback_latlong)

# rospy.Subscriber("/novatel/oem7/bestvel",BESTVEL, callback_vel)

heading=0    

rospy.Subscriber("/novatel/oem7/inspva",INSPVA, callback_heading)


def return_to_zero():
	global steering_angle
	global steer_dir
	global steer_rate
	
	#print("steering_angle:",steering_angle)
	while abs(steering_angle - steering_center) > center_threshold	:
	#while (True):    
		print("in return_to_zero function")
		print(steering_angle)
		# print("ffffffffaaaaaaaa: ",aeb_flag)
		if (steering_angle < steering_center):
			#dc=10
			#rotate_right()
			steer_rate = 15 + (abs(steering_angle - steering_center)/2) # Proportional to read angle
			#print("steer_rate:",steer_rate)
			steer_dir = 1
			
			#time.sleep(0.1)
		if (steering_angle > steering_center):
			#rotate_left()
			steer_rate = 15 +(abs(steering_angle - steering_center)/2) # Proportional to read angle
			#print("steer_rate:",steer_rate)
			steer_dir = -1
		if (steer_rate>80):
			steer_rate=80
		comm_ni()     
	stop_steering()


def send_data(ip_address, port, message,mode_delay):
    try:
        # Create a TCP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Connect to the IP address and port
        sock.connect((ip_address, port))
        token  = 0
        while token < 25*mode_delay:
            # Send the message
            data = sock.send(message.encode())
            token += data 
            
            # Delay for 1 second
            time.sleep(1)
        
    except Exception as e:
        print('An error occurred while sending data:', str(e))
    
    finally:
        # Close the socket
        sock.close()



# Example usage
ip_address = '169.254.178.227'
port = 5001
auto_ready = "M,N,0,0,0,0,0,0,0,0,0\r\n"
flag_one = "A3,D,0,1,20,0,0,0,0,0,0\r\n"
flag_one_five = "A3,D,0,1,40,0,0,0,0,0,0\r\n"
flag_zero = "A3,D,0,1,80,0,0,0,0,0,0\r\n"
flag_release = "A3,0,0,1,0,0,0,0,0,0,0\r\n"

# Call the send_data function
send_data(ip_address, port, auto_ready,1)
while not rospy.is_shutdown():
    print("outside.....................", aeb_flag)
    while aeb_flag != 2 :
        print(aeb_flag)
        if (aeb_flag==1):
            send_data(ip_address, port, flag_one,1)
        elif (aeb_flag==0.5):   #  3<ttc<8
            send_data(ip_address, port, flag_one_five,1)
        elif (aeb_flag==0.0 or aeb_flag==1.5): # ttc<3
            send_data(ip_address, port, flag_zero,1)

    send_data(ip_address, port, flag_release,4)
    # send_data(ip_address, port, flag_release,1)

'''
message = "A,N,0,1,50,0,0,0,0,0,0\r\n"

send_data(ip_address, port, message,1)

message = "A,D,0,-1,50,0,0,0,0,0,0\r\n"

send_data(ip_address, port, message,1)

send_data(ip_address, port, message,1)

# message = "A,D,10,0,0,0,0,0,0,0,0\r\n"

# send_data(ip_address, port, message,2)
# message = "A,D,40,0,0,0,0,0,0,0,0\r\n"

# send_data(ip_address, port, message,3)
message = "A,N,0,0,50,0,0,0,0,0,0\r\n"

send_data(ip_address, port, message,3)
# message = "A,N,0,0,0,0,0,0,0,0,0\r\n"

# send_data(ip_address, port, message,4)'''