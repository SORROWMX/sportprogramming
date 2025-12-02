import rospy
import math
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import pigpio
from geometry_msgs.msg import Point, PointStamped
from clover import srv
from std_srvs.srv import Trigger
from clover.srv import SetLEDEffect
from sensor_msgs.msg import Range

rospy.init_node('platform_delivery')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

pi = pigpio.pi()
SERVO_PIN = 18
pi.set_mode(SERVO_PIN, pigpio.OUTPUT)

search_height = 1.5
drop_height = 0.3
id_aruco = 100
h_chl = 2.0

def range_callback(msg):
    global h_chl
    h_chl = msg.range

rospy.Subscriber('rangefinder/range', Range, range_callback)

def navigate_wait(x=0, y=0, z=1, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            break
        rospy.sleep(0.2)

def get_body_pose(frame_id):
    try:
        trans = tfBuffer.lookup_transform(frame_id, "body", rospy.Time())
    except:
        return None
    pnt = tf2_geometry_msgs.do_transform_point(PointStamped(point=Point(x=0, y=0, z=0)), trans)
    return np.array([pnt.point.x, pnt.point.y, pnt.point.z])

def get_marker_pose(marker_id, frame_id="aruco_map"):
    try:
        trans = tfBuffer.lookup_transform(frame_id, "aruco_{}".format(marker_id), rospy.Time())
    except:
        return None
    pnt = tf2_geometry_msgs.do_transform_point(PointStamped(point=Point(x=0, y=0, z=0)), trans)
    return np.array([pnt.point.x, pnt.point.y, pnt.point.z])

def open():
    pi.set_servo_pulsewidth(SERVO_PIN, 800)
    set_effect(r=255, g=255, b=255)
    rospy.sleep(1)
    set_effect()

def close():
    pi.set_servo_pulsewidth(SERVO_PIN, 2300)
    set_effect(r=0, g=0, b=255)
    rospy.sleep(1)
    set_effect()

def search_platform():
    start_time = rospy.get_time()
    
    while rospy.get_time() - start_time < 10.0:
        platform_pos = get_marker_pose(id_aruco)
        if platform_pos is not None:
            return True
        rospy.sleep(0.5)
    
    return False

def center_over_platform():
    set_effect(r=255, b=255)
    
    FRQ = 15
    r = rospy.Rate(FRQ)
    prev_pos = None
    prev_vel = None
    prev_t = rospy.get_time()
    st_t = rospy.get_time()
    d = 10
    
    while (d > 0.15 or (rospy.get_time() - st_t < 3.0)) and not rospy.is_shutdown():
        pb = get_body_pose("aruco_map")
        platform_pos = get_marker_pose(id_aruco)
        
        now = rospy.get_time()
        
        if prev_pos is None:
            if pb is None or platform_pos is None:
                r.sleep()
                continue
            navigate(x=pb[0], y=pb[1], z=search_height, speed=0.5, frame_id="aruco_map")
            prev_pos = platform_pos[:2]
            prev_t = now
        else:
            if pb is not None and platform_pos is not None:
                d = np.linalg.norm(pb[:2] - platform_pos[:2])
            
            if platform_pos is not None:
                vel = (platform_pos[:2] - prev_pos) / (now - prev_t + 0.00001)
                vel = np.clip(vel, -0.5, 0.5)
                
                if np.linalg.norm(vel) < 0.045:
                    vel = np.array([0.0, 0.0])
                
                if prev_vel is not None:
                    vel = vel * 0.9 + prev_vel * 0.1
                
                target = platform_pos[:2] + vel * (1.0 / FRQ) * 1.0
                set_position(x=target[0], y=target[1], z=search_height, frame_id="aruco_map")
                
                prev_pos = platform_pos[:2].copy()
                prev_vel = vel.copy()
                prev_t = now
            else:
                if pb is not None:
                    navigate(x=pb[0], y=pb[1], z=search_height, frame_id="aruco_map")
        
        r.sleep()
    
    rospy.sleep(0.5)

def precise_landing_on_platform():
    set_effect(r=255, g=255)
    
    FRQ = 30
    r = rospy.Rate(FRQ)
    z_current = search_height
    z_vel = 0.15
    st_t = rospy.get_time()
    
    while h_chl > drop_height and not rospy.is_shutdown():
        pb = get_body_pose("aruco_map")
        platform_pos = get_marker_pose(id_aruco)
        
        if pb is not None:
            z_current = search_height - (rospy.get_time() - st_t) * z_vel
            z_current = max(z_current, drop_height)
            
            if platform_pos is not None:
                set_position(x=platform_pos[0], y=platform_pos[1], z=z_current, frame_id="aruco_map")
                print("h={:.2f}".format(h_chl))
            else:
                set_position(x=pb[0], y=pb[1], z=z_current, frame_id="aruco_map")
        
        r.sleep()
    
    rospy.sleep(0.5)

def main():
    close()
    navigate_wait(z=1.5, speed=1.0, frame_id='body', auto_arm=True)
    rospy.sleep(2)
    
    navigate_wait(x=1, y=3, z=search_height, speed=0.5)
    
    if not search_platform():
        navigate_wait(x=0, y=0, z=search_height)
        land()
        return
    
    center_over_platform()
    precise_landing_on_platform()
    
    open()
    set_effect(g=255)
    rospy.sleep(1)
    
    navigate_wait(z=1.0, speed=1.0, frame_id='body')
    navigate_wait(x=0, y=0, z=1.0)
    land()
    set_effect()
    
    pi.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        set_effect()
        pi.stop()
    except Exception as e:
        print("Error: {}".format(e))
        set_effect()
        pi.stop()
