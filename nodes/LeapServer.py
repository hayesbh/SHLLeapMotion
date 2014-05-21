#!/usr/bin/env python
import roslib; roslib.load_manifest("LeapMotion")
import sys
import rospy
import core_skill_execution_server
from core_skill_execution_server.srv import *

sys.path.insert(0, "./lib/")
import Leap
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

def ExecuteSkill(robot, skill_name):
  print "Executing %s on robot: %s", skill_name, robot
  if (robot != active_robot_name_):
    SetActiveRobot(robot)
  srv = ControlCommandRequest()
  srv.command_type = srv.COMMAND_LOAD_SKILL
  srv.command_string_args.append(skill_name)
  resp = skill_server_control_service_(srv)
  if (not resp.command_ok):
    print "Couldn't load skill %s on robot %s" % skill_name, robot
  return False

  srv = ControlCommandRequest()
  srv.command_type = srv.COMMAND_EXECUTE_SKILL
  resp = skill_server_control_service_(srv)
  if (not resp.command_ok):
    print "Couldn't execute skill %s on robot %s" % skill_name, robot
    return False
  return True


def SetActiveRobot(robot_name):
  active_robot_name_ = robot_name
  InitializeSkillServerPublisher()
  print "LeapServer set active robot to %s", active_robot_name_
  # Tell skill_server that active robot is robot_name

def InitializeSkillServerPublisher():
  skill_server_service_name = "/"+active_robot_name_+"/skillserver_control_command"
  print "Connecting to core_skill_execution_server...",
  rospy.wait_for_service(skill_server_service_name)
  print "Connected"

  try:
    skill_server_control_service_ = rospy.ServiceProxy(skill_server_service_name, ControlCommand)
  except rospy.ServiceException, e:
    print "ControlCommand call failed: %s" % e



class GestureListener(Leap.Listener):
  def on_init(self, controller):
    print "GestureListener on_init"

  def on_connect(self, controller):
    print "GestureListener on_connect"
    controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE)
    controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP)
    controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP)
    controller.enable_gesture(Leap.Gesture.TYPE_SWIPE)

  def on_disconnect(self, controller):
    print "GestureListener on_disconnect"

  def on_frame(self, controller):
    frame = controller.frame()

    print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
          frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

  def state_string(self, controller):
    print "GestureListener state_string"
    if state == Leap.Gesture.STATE_START:
      return "STATE_START"

    if state == Leap.Gesture.STATE_UPDATE:
      return "STATE_UPDATE"

    if state == Leap.Gesture.STATE_STOP:
      return "STATE_STOP"

    if state == Leap.Gesture.STATE_INVALID:
      return "STATE_INVALID"

def main():
	# Initialize Main Program Vars
	skill_server_control_service_ = None
	active_robot_name_ = sys.argv[1] if len(sys.argv) > 1 else 'GLaDOS'

	rospy.init_node('LeapMotionServer')

	# Initialize Controller Service to core_skill_execution_server
	# SetActiveRobot(active_robot_name_)

	# Initialize connection to LeapMotion
	leap_listener_ = GestureListener()
	leap_controller_ = Leap.Controller()
	leap_controller_.add_listener(leap_listener_)

	# Gesture Recognition Loop
	while not rospy.is_shutdown():
		pass

	leap_controller_.remove_listener(leap_listener_)	


if __name__ == '__main__':
	main()
