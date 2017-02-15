import rospy
import smach
import smach_ros
import types
import tf
import math
import sys, getopt
import subprocess

from actionlib import *
from actionlib.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from asr_flir_ptu_controller.msg import PTUMovementGoal, PTUMovementAction


def main(argv):
    rospy.init_node("ptu_manual_controller_script")

    rospy.loginfo('Trying to communicate with ptu via action msg.')
    client = actionlib.SimpleActionClient('ptu_controller_actionlib', 
                                          PTUMovementAction)
    if not client.wait_for_server(rospy.Duration.from_sec(10)):
        rospy.logwarn("Could not connect to ptu action server")
        return 'aborted'
      
    #path to visualization script, which is called after each ptu_movement

    visualization_script = ''
    try:
      opts, args = getopt.getopt(argv,"v:",["visualization="])
      for opt, arg in opts:
	if opt in ("-v", "--visualization"):
	  visualization_script = arg
	  print 'Using ' + visualization_script + ' for visualization.'
    except getopt.GetoptError:
      print 'No visualization script specified.'

    while True:
        pan = float(raw_input("Pan? "))
        tilt = float(raw_input("Tilt? "))
        ptu_goal = PTUMovementGoal()
        ptu_goal.target_joint.header.seq = 0
        ptu_goal.target_joint.name = ['pan', 'tilt']
        ptu_goal.target_joint.velocity = [0, 0]
        ptu_goal.target_joint.position = [pan, tilt] 
        client.send_goal(ptu_goal)
        if visualization_script != '':
	  subprocess.call(" python " + visualization_script + " 1", shell=True)

    rospy.spin()

if __name__ == '__main__':
    try:
         main(sys.argv[1:])
    except rospy.ROSInterruptException: pass
    
