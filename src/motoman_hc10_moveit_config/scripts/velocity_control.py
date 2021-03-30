#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import roslib; roslib.load_manifest('motoman_driver')
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import matplotlib.pyplot as plt
import time
from sensor_msgs.msg import JointState

sTab = []
spTab = []
q1 = []
q2 = []
q3 = []
q4 = []
q5 = []
q6 = []

class MoveGroupPythonIntefaceTutorial(object):
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test')

    # Misc variables
    self.group = moveit_commander.MoveGroupCommander('manipulator')
    self.box_name = ''
    self.acc = 0
    self.speed = 0
    self.t1 = 0
    self.t2 = 0
    self.tmax = 0

  def set_acc_and_vel_values(self, acc, speed):
    self.acc = acc
    self.speed = speed

  def calculate_t_values(self, A, B):
    self.t1 = self.speed/self.acc
    self.t2 = self.get_distance(A, B)/self.speed
    self.tmax = self.t1 + self.t2

  def get_jacobian_matrix(self, joint_states):
    matrix = np.array(self.group.get_jacobian_matrix(joint_states))
    return matrix

  def get_inverse_jacobian_matrix(self, joint_states):
    return np.linalg.pinv(self.get_jacobian_matrix(joint_states))

  def get_distance(self, A, B):
    return np.sqrt( (A[0]-B[0])**2 + (A[1]-B[1])**2 )

  def get_coord_linear(self, A, B, s): # M(s)
    M = [0, 0]
    M[0] = A[0] + (B[0]-A[0])*s
    M[1] = A[1] + (B[1]-A[1])*s

    return M

  def get_coord_circular(self, A, B, s):
    M = [0, 0]
    M[0] = (A[0]+B[0])/2 + self.get_distance(A, B)/2*-np.cos(s*np.pi)
    M[1] = (A[1]+B[1])/2 + self.get_distance(A, B)/2*np.sin(s*np.pi)
    return M

  def get_velocity(self, A, B, t): # dot s(t)
    if 0 <= t <= self.t1:
      return self.acc*t
    elif self.t1 < t <= self.t2:
      return self.speed
    elif self.t2 < t <= self.tmax:
      return -self.acc*t + self.acc*self.tmax

  def get_position(self, A, B, t): # s(t)
    if 0 <= t <= self.t1:
      return self.acc*(t**2)/2
    elif self.t1 < t <= self.t2:
      return self.speed*t - self.t1*self.speed/2
    elif self.t2 < t <= self.tmax:
      return -self.acc*(t**2)/2 + self.acc*self.tmax*t + self.get_distance(A, B) - self.acc*(self.tmax**2)/2

  def get_coord_position(self, A, B, t): # M(t)
    global sTab
    s = self.get_position(A, B, t)
    sTab.append(s)
    return self.get_coord_linear(A, B, s)

  def get_coord_velocity(self, A, B, t): # dot M(t)
    global spTab
    dot_s = self.get_velocity(A, B, t)
    spTab.append(dot_s)
    return self.get_coord_linear(A, B, dot_s)

  def sample_velocities(self, A, B, Te):
    global q1, q2, q3, q4, q5, q6

    tTab = np.arange(0, self.tmax, Te)
    qTab = np.zeros((len(tTab), 6))
    dqTab = np.zeros((len(tTab), 6))
    qTab[0,:] = [-2.996920347213745, 0.4123761057853699, 1.4188867807388306, 0.01778588630259037, -0.9282400012016296, 1.8107110261917114]
    dqTab[0,:] = [0]*6

    q1.append(qTab[0,0])
    q2.append(qTab[0,1])
    q3.append(qTab[0,2])
    q4.append(qTab[0,3])
    q5.append(qTab[0,4])
    q6.append(qTab[0,5])
    
    self.get_coord_position(A, B, tTab[0])
    self.get_coord_velocity(A, B, tTab[0])

    for i in range(1, len(tTab)):
      pos = self.get_coord_position(A, B, tTab[i])
      vel = self.get_coord_velocity(A, B, tTab[i])

      pTab = np.zeros((6,1))
      pTab[0,0] = pos[0]
      pTab[1,0] = pos[1]
      pTab[2,0] = 0.152176 
      pTab[3,0] = 175.8515*np.pi/180
      pTab[4,0] = 1.9002*np.pi/180
      pTab[5,0] = -67.4549*np.pi/180

      vTab = np.zeros((6,1))
      vTab[0,0] = vel[0]
      vTab[1,0] = vel[1]

      invJ = self.get_inverse_jacobian_matrix(list(qTab[i-1,:]))

      dqTab[i,:] = np.transpose(np.matmul(invJ, vTab))
      
      qTab[i,:] = np.add(qTab[i-1,:], np.dot(dqTab[i,:], Te))
      qTab[i,0] = (qTab[i,0] + np.pi)%(2*np.pi) - np.pi
      qTab[i,1] = (qTab[i,1] + np.pi)%(2*np.pi) - np.pi
      qTab[i,2] = (qTab[i,2] + 5*np.pi/180)%(2*np.pi) - 5*np.pi/180
      qTab[i,3] = (qTab[i,3] + np.pi)%(2*np.pi) - np.pi
      qTab[i,4] = (qTab[i,4] + np.pi)%(2*np.pi) - np.pi
      qTab[i,5] = (qTab[i,5] + np.pi)%(2*np.pi) - np.pi
      
      q1.append(qTab[i,0])
      q2.append(qTab[i,1])
      q3.append(qTab[i,2])
      q4.append(qTab[i,3])
      q5.append(qTab[i,4])
      q6.append(qTab[i,5])

    return (tTab, qTab, dqTab)

  def build_traj(self, tTab, qTab, dqTab, Te):
    points = []

    start_pt = JointTrajectoryPoint()
    start_pt.positions = [-2.996920347213745, 0.4123761057853699, 1.4188867807388306, 0.01778588630259037, -0.9282400012016296, 1.8107110261917114]
    start_pt.velocities = [0]*6
    start_pt.time_from_start = rospy.Duration(0.0)
    points.append(start_pt)

    for i in range(1, len(tTab)):
      points.append(JointTrajectoryPoint())
      points[i].positions = list(qTab[i,:])
      points[i].velocities = [0]*6
      points[i].time_from_start = rospy.Duration(Te*i)

    return JointTrajectory(joint_names=self.group.get_joints()[:-1], points=points)

  def wait_for_subs(self, pub, num_subs, min_time, timeout):
    end = rospy.Time.now() + rospy.Duration(timeout)
    rospy.sleep(min_time)

    r = rospy.Rate(10)
    while (pub.get_num_connections() < num_subs) and (rospy.Time.now() < end) and not rospy.is_shutdown():
      r.sleep()

    return (pub.get_num_connections() >= num_subs)

  def move_to_joint(self, tTab, qTab, dqTab, Te):
    traj = self.build_traj(tTab, qTab, dqTab, Te)

    pub = rospy.Publisher('joint_path_command', JointTrajectory, queue_size=1)

    if not self.wait_for_subs(pub, 1, 0.5, 2.0):
      rospy.logwarn('Timeout while waiting for subscribers.  Publishing trajectory anyway.')

    pub.publish(traj)

  def initPosition(self, A, B, duration = 3): 
    start_pt = JointTrajectoryPoint()
    start_pt.positions = self.group.get_current_joint_values()
    start_pt.velocities = [0]*6
    start_pt.time_from_start = rospy.Duration(0.0)

    end_pt = JointTrajectoryPoint()
    end_pt.positions = [-2.996920347213745, 0.4123761057853699, 1.4188867807388306, 0.01778588630259037, -0.9282400012016296, 1.8107110261917114]
    end_pt.velocities = [0]*6
    end_pt.time_from_start = rospy.Duration(duration)
    traj = JointTrajectory(joint_names=self.group.get_joints()[:-1], points=[start_pt, end_pt])

    pub = rospy.Publisher('joint_path_command', JointTrajectory, queue_size=1)
    if not self.wait_for_subs(pub, 1, 0.5, 2.0):
      rospy.logwarn('Timeout while waiting for subscribers.  Publishing trajectory anyway.')

    pub.publish(traj)
    time.sleep(duration)

def main():
  try:
    hc10 = MoveGroupPythonIntefaceTutorial()
    
    A = [-0.688304, -0.260798]
    B = [-0.688294, -0.415200]
    acc = 0.1
    speed = 0.05
    Te = 0.01
    hc10.set_acc_and_vel_values(acc, speed)
    hc10.calculate_t_values(A, B)

    hc10.initPosition(A, B)
    qpos = [0,0,0,0,0,0]

    (tTab, qTab, dqTab) = hc10.sample_velocities(A, B, Te)
    hc10.move_to_joint(tTab, qTab, dqTab, Te)
    
    # Plot figures
    plt.figure()
    plt.plot(tTab, q1, ".-", label="q1")
    plt.plot(tTab, q2, ".-", label="q2")
    plt.plot(tTab, q3, ".-", label="q3")
    plt.plot(tTab, q4, ".-", label="q4")
    plt.plot(tTab, q5, ".-", label="q5")
    plt.plot(tTab, q6, ".-", label="q6")
    plt.xlabel("Temps (s)")
    plt.ylabel("Joints (rad)")
    plt.title("Variations des joints")
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(tTab, sTab)
    plt.xlabel("Temps (s)")
    plt.ylabel("Distance parcourue (m)")
    plt.title("Profil de distance")
    plt.grid()

    plt.figure()
    plt.plot(tTab, spTab)
    plt.xlabel("Temps (s)")
    plt.ylabel("Vitesse (m/s)")
    plt.title("Profil de vitesse")
    plt.grid()
    
    plt.show()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
