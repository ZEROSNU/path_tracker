#!/usr/bin/env python
import rospy
import numpy as np
import math
from core_msgs.msg import Path3DArray
from core_msgs.msg import control
from core_msgs.msg import CenPoint
from core_msgs.msg import ser_com
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32


class tracker : 
      def __init__ (self) :
            self.pub_rate = 20.0 #Hz
            self.time_acc_ratio = 50 #loop in rate * acc_ratio
            self.buff_size = 10
            self.speed_slope = 0.25 #m/s/conf
            self.delta_y_max = 0.2 #m maximum error
            self.speed_max = 1.5 #m/s
            self.speed_avg = 0.01
            # -------------^CONSTANT^--------------

            self.L = 1.54 #m distance between two wheel axis
            self.lr = 0.53 #m  distance between cm and rear wheel
            self.cencorr = -0.04 #m  origin distance between cenPoint and sPath
            #car coordinate origin will be the geometrical center of car frame, except bumper.
            #goalpoint should be cenPoint.y_waypoint + self.cencorr

            # ----------------------------------^HARDWARE_CONSTANT^------------------------------

            
            self.control = control() #my order to the car
            self.goalpoint = CenPoint() #data of goal point - sPath origin placed in center, cenPath origin placed 770 from front wheel
            self.cont_buff = np.empty(self.buff_size, dtype=(type([control(), float()]))) #past buff of the ordering
            
            self.cartime = 0.0 #time variables _ time when this node get sPath or cenPoint
            self.stime = 0.0
            self.ctime = 0.0
            self.sttime = 0.0
            #-----------------------^DETERMINED^---------------------


            self.sPath = Path3DArray()
            self.cPoint = CenPoint()
            self.sDelay = 0.0
            self.cDelay = 0.0
            self.obs = 0 
            self.state_buff = np.empty(self.buff_size, dtype=type([ser_com(), float()]))
            #-------^INPUT^---------------
            
      #function needs to update goalpoint - get speed and steering at time t
      def get_state(self, time) :
            t_state = ser_com()
            for i in reversed(self.state_buff) :
                  if i == None :
                        print "no data in state_buff"
                        break
                  elif time < i[1] :
                        t_state.is_auto = i[0].is_auto
                        t_state.estop = i[0].estop
                        t_state.gear = i[0].gear
                        t_state.brake = i[0].brake
                        t_state.speed = i[0].speed
                        t_state.steer = i[0].steer
                        t_state.encoder = i[0].encoder
                        t_state.alive = i[0].alive
                  else :
                        break
            return t_state

      def update_goalpoint(self, intime, curtime) : # the time when the goalpoint was input
            ugoalpoint = CenPoint()
            ugoalpoint.x_waypoint = self.goalpoint.x_waypoint
            ugoalpoint.y_waypoint = self.goalpoint.y_waypoint
            ugoalpoint.confidence = self.goalpoint.confidence
            time_ind = intime
            init = False
            tmp = [ser_com(), float()]
            for ind in self.state_buff :
                  if ind == None :
                        continue
                  elif ind[1] - time_ind >=0 :
                        if init == False :
                              beta = np.arctan(self.lr / self.L * np.tan( -self.get_state(time_ind).steer * 180 / np.pi))
                        else :
                              beta = np.arctan(self.lr / self.L * np.tan( -tmp[0].steer * 180 / np.pi))
                        if beta != 0 :
                              R = self.lr / np.sin(beta)
                              if init == False :
                                    yaw_rate = self.get_state(time_ind).speed / R
                              else :
                                    yaw_rate = tmp[0].speed / R
                              delta_t = time_ind - ind[1]
                              delta_phi = delta_t * yaw_rate
                              x_prime = ugoalpoint.x_waypoint - R * np.sin(delta_phi)
                              y_prime = ugoalpoint.y_waypoint - R * (1 - np.cos(delta_phi))
                              ugoalpoint.x_waypoint = np.cos(delta_phi) * x_prime + np.sin(delta_phi) * y_prime
                              ugoalpoint.y_waypoint = - np.sin(delta_phi) * x_prime + np.cos(delta_phi) * y_prime
                        
                        else :
                              if init == False :
                                    ugoalpoint.x_waypoint = ugoalpoint.x_waypoint - self.get_state(time_ind).speed * (time_ind - ind[1])
                              else :
                                    ugoalpoint.x_waypoint = ugoalpoint.x_waypoint - tmp[0].speed * (time_ind - ind[1])
                  
                        time_ind = ind[1]
                        tmp[0] = ind[0]
                        tmp[1] = ind[1]
                        init = True
                              
            if init == False :
                  return
            
            beta = np.arctan(self.lr / self.L * np.tan( -self.get_state(time_ind).steer * 180 / np.pi))
            if beta!=0 :
                  R = self.lr / np.sin(beta)
                  yaw_rate = self.get_state(curtime).speed / R
                  delta_t = curtime - ind[1]
                  delta_phi = delta_t * yaw_rate
                  x_prime = ugoalpoint.x_waypoint - R * np.sin(delta_phi)
                  y_prime = ugoalpoint.y_waypoint - R * (1 - np.cos(delta_phi))
                  ugoalpoint.x_waypoint = np.cos(delta_phi) * x_prime + np.sin(delta_phi) * y_prime
                  ugoalpoint.y_waypoint = - np.sin(delta_phi) * x_prime + np.cos(delta_phi) * y_prime
            else :
                  ugoalpoint.x_waypoint = ugoalpoint.x_waypoint - self.get_state(curtime).speed * (time_ind - ind[1])
            self.goalpoint = ugoalpoint
      #how to update goalpoint integrate state_buff 0 to n-1

      def update_cartime(self, time) :
            self.cartime += time

      def decide_sPath_n(self) :
            return 20
      #need to decide n for s_path


      def calc_get_cont(self) :
            self.control = control()
            pttype = ''
            t=0.0


            if self.ctime>0 and self.obs==0 :
                  pttype = 'c'
                  print "publish based on cPath"
                  self.goalpoint.x_waypoint = self.cPoint.x_waypoint - self.cencorr
                  self.goalpoint.y_waypoint = self.cPoint.y_waypoint
                  self.goalpoint.confidence = self.cPoint.confidence
                  t=self.ctime
            elif self.stime>0:
                  pttype = 's'
                  print "publish based on sPath"
                  n = self.decide_sPath_n()
                  self.goalpoint.x_waypoint = 0.3*(200-self.sPath.pathpoints[n].y) #0.3m per pixel
                  self.goalpoint.y_waypoint = 0.3*(100-self.sPath.pathpoints[n].x)
                  self.goalpoint.confidence = 1.0*n/20 #this may be changed
                  t=self.stime

            else :
                  print "no input"
                  return self.control
            

            if pttype == 'c' :
                  self.update_goalpoint(self.ctime, self.cartime)
            elif pttype == 's' :
                  self.update_goalpoint(self.stime, self.cartime)

            ld = math.sqrt(self.goalpoint.x_waypoint*self.goalpoint.x_waypoint + self.goalpoint.y_waypoint*self.goalpoint.y_waypoint)
            delta = -np.arctan(2 * self.L * self.goalpoint.y_waypoint / ld / (ld + 2 * self.lr * self.goalpoint.x_waypoint/ld)) # ld is lookahead distance
            self.control.is_auto = 0
            self.control.estop = 0
            self.control.gear = 0
            if pttype == 'c' :
                  self.control.speed = self.goalpoint.confidence * self.speed_slope *0.04 #slow down factor
            elif pttype == 's' :
                  self.control.speed = self.speed_avg
            self.control.steer = delta / np.pi * 180 #in degree
            self.control.brake = 0
            self.cont_buff[0:-1]=self.cont_buff[1:self.buff_size]
            self.cont_buff[self.buff_size-1] = [self.control, self.cartime]
            if self.control.speed > self.speed_max :
                  self.control.speed = 0
            return self.control #need to update self.control

      #need function for PID gain



      #function that write, show, get the data of class
      def write_obs(self, data) :
            self.obs=data.data
      
      def write_cPoint(self, data) :
            self.cPoint=data
            self.ctime=self.cartime

      def write_sPath(self, data) :
            self.sPath=data
            self.stime=self.cartime

      def write_stbuff(self, data) :
            self.sttime = self.cartime
            self.state_buff[0:-1]=self.state_buff[1:self.buff_size]
            self.state_buff[self.buff_size-1]=[data, self.sttime]
            
      def show_sPath(self) :
            for i in self.sPath.pathpoints :
                  print i
            print self.stime
      
      def show_cPoint(self) :
            print self.ctime
            print self.cPoint

      def show_obs(self) :
            print self.obs

      def show_stbuff(self) :
            print self.state_buff

      def show_cont_buff(self) :
            for i in self.cont_buff :
                  print i

      def show_control(self) :
            print self.control
      
      def get_sPath(self) :
            return self.sPath

      def get_cPoint(self) :
            return self.cPoint

      def get_obs(self) :
            return self.obs

      def get_cont_buff(self, i) :
            return self.cont_buff[i]
      
      def get_control(self) :
            return self.control

main_track = tracker()

def callback_s(data) :
      main_track.write_sPath(data)
#      main_track.show_sPath()
#      print "Got sPath"

def callback_c(data) :
      main_track.write_cPoint(data)
#      main_track.show_cPoint()
#      print "Got cPoint"

def callback_obs(data) :
      main_track.write_obs(data)
#      main_track.show_obs()
#      print "Got obs "

def callback_stbuff(data) :
      main_track.write_stbuff(data)
#      main_track.show_stbuff()
#      print "Got state_buff"

def init() :
      
      rospy.init_node('path_tracker', anonymous=True)
      rospy.Subscriber('/sPath', Path3DArray, callback_s)
      rospy.Subscriber('/cPoint', CenPoint, callback_c)
      rospy.Subscriber('/flag_obstacle', Int32, callback_obs)
      rospy.Subscriber('/serial_topic', ser_com, callback_stbuff)
      
      pub = rospy.Publisher('/control', control, queue_size=10)
      rate = rospy.Rate(main_track.pub_rate*main_track.time_acc_ratio)
      i=0
      while not rospy.is_shutdown() :
            if i % main_track.time_acc_ratio == 0:
                  pub.publish(main_track.calc_get_cont())
                  i=0
            rate.sleep()
            main_track.update_cartime(1.0/main_track.pub_rate/main_track.time_acc_ratio)
            i+=1


if __name__ == '__main__':
      init()
