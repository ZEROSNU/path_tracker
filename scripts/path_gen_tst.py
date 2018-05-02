#!/usr/bin/env python
import rospy
import numpy as np
from core_msgs.msg import Path3DArray
from core_msgs.msg import CenPoint
from core_msgs.msg import ser_com
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32

def test_map_gen() :
  pub_c = rospy.Publisher('/cPoint', CenPoint, queue_size = 10)
  pub_s = rospy.Publisher('/sPath', Path3DArray, queue_size = 10)
  pub_f = rospy.Publisher('/flag_obstacle', Int32, queue_size = 10)
  pub_state = rospy.Publisher('/serial_topic',ser_com, queue_size = 10)
  rospy.init_node('path_gen_tst', anonymous=True)

  rate = rospy.Rate(20) #1Hz
  time=0
  while not rospy.is_shutdown():
    time+=1
    path = Path3DArray()
    state = ser_com()
    for i in range(100) :
      v = Vector3()
      v.x=50.0
      v.z=0.0
      v.y=float(i)
      path.pathpoints.append(v)
      path.id.append(0)
      path.delta.append(0.0)

    point = CenPoint()
    point.x_waypoint=0.0
    point.y_waypoint=25-(time%50)
    point.confidence=time%10
    rospy.loginfo(point)
    pub_c.publish(point) #publishing cpoint


    state.speed=1
    state.steer=45
#    rospy.loginfo(state)
    pub_state.publish(state) #publishing state

    if time%4==0:
      pub_s.publish(path)
#      rospy.loginfo(path)
    
    if time%100<50: #publishing obstacle
      pub_f.publish(0)
#      rospy.loginfo(0)
    else:
      pub_f.publish(time%100-50)
#      rospy.loginfo(time%100-50)
    rate.sleep()

if __name__ == '__main__':
  try :
    test_map_gen()
  except rospy.ROSInterruptException:
    pass

