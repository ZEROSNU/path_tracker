#!/usr/bin/env python
import rospy
import numpy as np
from core_msgs.msg import PathArray
from core_msgs.msg import CenPoint
from core_msgs.msg import VehicleState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32

def test_map_gen() :
  pub_c = rospy.Publisher('/waypoints', CenPoint, queue_size = 10)
  pub_s = rospy.Publisher('/sPath', PathArray, queue_size = 10)
  pub_f = rospy.Publisher('/flag_obstacle', Int32, queue_size = 10)
  pub_state = rospy.Publisher('/serial_topic',VehicleState, queue_size = 10)
  pub_em = rospy.Publisher('/emergency_stop', Int32, queue_size = 10)

  rospy.init_node('path_gen_tst', anonymous=True)

  rate = rospy.Rate(20) #1Hz
  time=0
  while not rospy.is_shutdown():
    time+=1
    path = PathArray()
    state = VehicleState()
    for i in range(50) :
      v = Vector3()
      v.x=50.0
      v.z=0.0
      v.y=float(i)
      path.pathpoints.append(v)
      path.headings.append(0.0)
    path.header.stamp = rospy.Time.from_seconds(rospy.Time.now().to_sec())

    point = CenPoint()
    point.x_waypoint=25-(time%50)
    point.y_waypoint=0.0
    point.confidence=time%10
#    rospy.loginfo(path)
    pub_c.publish(point) #publishing cpoint


    state.speed=2
    state.steer=-10
#    rospy.loginfo(state)
    pub_state.publish(state) #publishing state

    #if time%4==0:
      #pub_s.publish(path)
#      rospy.loginfo(path)

    # if time%100<50: #publishing obstacle
      #pub_f.publish(0)
#      rospy.loginfo(0)
    # else:
      #pub_f.publish(time%100-50)
#      rospy.loginfo(time%100-50)
    if time%500 == 0 :
      pub_em.publish(1)
    rate.sleep()

if __name__ == '__main__':
  try :
    test_map_gen()
  except rospy.ROSInterruptException:
    pass
