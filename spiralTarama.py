import mailcap
from site import venv
import dronekit_sitl
from dronekit import Command,connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import cv2
import rospy
from geometry_msgs.msg  import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
import numpy as np
from std_srvs.srv import *
import math



sitl = dronekit_sitl.start_default()
connection_string = "127.0.0.1:14550"
print("Connecting to iha on: %s" % (connection_string))

iha = connect(connection_string, wait_ready=True)
cmds = iha.commands



def arm_and_takeoff(aTargetAltitude):
  
    while not iha.is_armable:
        print (" iha bekleniyor ")
        time.sleep(1)

    print ("Arming motors")
    
    iha.mode    = VehicleMode("GUIDED")
    iha.armed   = True

    
    while not iha.armed:
        print (" arm için bekleniyor")
        time.sleep(1)

    print ("Kalkış Başlıyor")
    iha.simple_takeoff(aTargetAltitude) 

    
    while True:
        print (" Yükseklik: ",iha.location.global_relative_frame.alt) 
        
        if iha.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("hedef yüksekliğe ulaşıldı")
            break
        time.sleep(1)


def goto_position_target_local_ned(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    iha.send_mavlink(msg)

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    iha.send_mavlink(msg)
    time.sleep(0.2)
    
    # send command to vehicle on 1 Hz cycle
   

def spiral():
    for i in range(0,600,3):
        vt = i/40*math.pi
        y=(vt*1+5)*math.sin(vt)
        x=(vt*1+5)*math.cos(vt)
        send_ned_velocity(y/15, x/15, 0)
        #goto_position_target_local_ned(y, x, 0)
        #time.sleep(1)

def circle():
    for i in range(0,600,8):
        vt = i/40*math.pi
        y=(vt*5+5)*math.sin(vt)
        x=(vt*5+5)*math.cos(vt)
        send_ned_velocity(y/10, x/10, 0, 1)
        #goto_position_target_local_ned(y, x, 0)
        #time.sleep(1)
    
def spiral2():
    for i in range(0,600):
        vt = i/40*math.pi
        y=(vt*1+5)*math.sin(vt)
        x=(vt*1+5)*math.cos(vt)
        send_ned_velocity(y/10, x/10, 0)
        #goto_position_target_local_ned(y, x, 0)
        #time.sleep(1)

def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance in kilometers between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a)) 
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles. Determines return value units.
    km = c*r
    m = km*1000
    return m

def spiralNav():
    x_init=iha.location.global_relative_frame.lat
    y_init=iha.location.global_relative_frame.lon
    alt = 10
    dpowX = pow(10,-6.5)
    dpowY = pow(10,-6.3)
    cmds.clear()
    time.sleep(1)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, alt))
    for i in range(1,900,3):
        vt = i/90*math.pi
        sp = i/90
        dy=(vt*5+5)*math.sin(vt)
        dx=(vt*5+5)*math.cos(vt)
        x = dx*dpowX/math.sqrt(sp+1) + x_init
        y = dy*dpowY/math.sqrt(sp+1) + y_init
        cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,x,y,alt))
        print(y)
        print(x)
    cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,x,y,alt))

    cmds.upload()
    time.sleep(1)
    iha.mode = VehicleMode("AUTO")
    print( "bitti")
# def spiralNav7():
#     x=iha.location.global_relative_frame.lat
#     y=iha.location.global_relative_frame.lon
#     alt = 10
#     dpow = pow(10,-7)
#     cmds.clear()
#     time.sleep(1)
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, alt))
#     for i in range(0,600):
#         vt = i/40*math.pi
#         dy=(vt*3+5)*math.sin(vt)
#         dx=(vt*3+5)*math.cos(vt)
#         x = dx*dpow + x
#         y = dy*dpow + y
#         cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,x,y,alt))
#         print(y)
#         print(x)
#     cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,x,y,alt))

#     cmds.upload()
#     time.sleep(1)
#     iha.mode = VehicleMode("AUTO")
def spiralNavCircle():
    x_init=iha.location.global_relative_frame.lat
    y_init=iha.location.global_relative_frame.lon
    r = 20
    N = 5
    dis = 25
    cycle = 30
    alt = 10
    dpow = pow(10,-5.8)
    cmds.clear()
    time.sleep(1)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, alt))
    for i in range(0,N):
        for j in range(0,cycle):
            t = (2*math.pi)*j/cycle
            sp = dis*j/cycle
            dr = sp + r 
            dy=dr*math.sin(t)
            dx=dr*math.cos(t)
            x = dx*dpow + x_init
            y = dy*dpow + y_init
            cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,x,y,alt))
            print(x)
            print(y)
        r += dis
    cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,x,y,alt))
    cmds.upload()
    time.sleep(1)
    iha.mode = VehicleMode("AUTO")

# def spiralNav2():
#     x_init=iha.location.global_relative_frame.lat
#     y_init=iha.location.global_relative_frame.lon
#     r = 50
#     alt = 10
#     dpow = pow(10,-6)
#     cmds.clear()
#     time.sleep(1)
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, alt))
#     for i in range(0,600):
#         vt = i/40*math.pi
#         dy=(vt)*math.sin(vt)
#         dx=(vt)*math.cos(vt)
        
#         x = dx*dpow + x_init
#         y = dy*dpow + y_init
#         cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,x,y,alt))
#         print(y)
#         print(x)
#     cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,x,y,alt))

#     cmds.upload()
#     time.sleep(1)
#     iha.mode = VehicleMode("AUTO")

arm_and_takeoff(10)
#spiralNav()

#spiralNav7()
spiralNavCircle()
#print(haversine(-35.36311117 ,149.16577231,-35.36349584 ,149.16614279))

#0.00001140 yaklaşık 1 metre
#0.00002 denenecek değer  1.7 metre

# 10^-6
		



#print(" Groundspeed: %s" % iha.groundspeed)    # settable
#print(" Airspeed: %s" % iha.airspeed)		
        
       
	