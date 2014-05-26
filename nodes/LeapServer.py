#!/usr/bin/env python
import roslib; roslib.load_manifest("LeapMotion")
import sys
import rospy
import core_skill_execution_server
from core_skill_execution_server.srv import *
import time

sys.path.insert(0, "./lib/")
import Leap
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

active_robot_name_ = ""
skill_server_control_service_ = None



def ExecuteSkill(robot, skill_name):
  global active_robot_name_, skill_server_control_service_
  print "Executing %s on robot: %s" % (skill_name, robot)

  if (robot != active_robot_name_):
    SetActiveRobot(robot)
  srv = ControlCommandRequest()
  srv.command_type = srv.COMMAND_LOAD_SKILL
  srv.command_string_args.append(skill_name)
  resp = skill_server_control_service_(srv)
  if (not resp.command_ok):
    rospy.logerror("Couldn't load skill %s on robot %s" % (skill_name, robot))
    return False

  srv = ControlCommandRequest()
  srv.command_type = srv.COMMAND_EXECUTE_SKILL
  rospy.loginfo("Beginning Skill Execution...")
  resp = skill_server_control_service_(srv)
  if (not resp.command_ok):
    rospy.logerror("Couldn't execute skill %s on robot %s" % (skill_name, robot))
    return False
  rospy.loginfo("...Completed Skill Execution")
  return True


def SetActiveRobot(robot_name):
  global active_robot_name_, skill_server_control_service_
  active_robot_name_ = robot_name
  rospy.loginfo("LeapServer setting active robot to %s..." % active_robot_name_)
  InitializeSkillServerPublisher()
  rospy.loginfo("...LeapServer set active robot to %s" % active_robot_name_)

def InitializeSkillServerPublisher():
  global active_robot_name_, skill_server_control_service_
  skill_server_service_name = "/"+active_robot_name_+"/skillserver_control_command"
  rospy.loginfo("Connecting to core_skill_execution_server (Service: %s)..." % skill_server_service_name)
  rospy.wait_for_service(skill_server_service_name)
  rospy.loginfo("Connected")

  try:
    skill_server_control_service_ = rospy.ServiceProxy(skill_server_service_name, ControlCommand)
  except rospy.ServiceException, e:
    rospy.loginfo("ControlCommand call failed: %s" % e)


def HandleSwipeGesture(swipe_classification):
  if (swipe_classification == "right"):
    SetActiveRobot("GLaDOS")
  elif (swipe_classification == "left"):
    SetActiveRobot("Wheatley")
  elif (swipe_classification == "up"):
    ExecuteSkill(active_robot_name_, "standup")
  elif (swipe_classification == "down"):
    ExecuteSkill(active_robot_name_, "sitdown")
  elif (swipe_classification == "forward"):
    ExecuteSkill(active_robot_name_, "pushback")
  elif (swipe_classification == "backwards"):
    ExecuteSkill(active_robot_name_, "pullforward")

def HandleCircleGesture(circle_classification):
  if (circle_classification == "clockwise"):
    ExecuteSkill(active_robot_name_, "rotatecw")
  elif (circle_classification == "counterclockwise"):
    ExecuteSkill(active_robot_name_, "rotateccw")

class GestureListener(Leap.Listener):
  last_gesture_time_ = 0

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

    #print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
    #      frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

    if (len(frame.gestures()) > 0 and time.time() - self.last_gesture_time_ > 2):
      print "Number of Gestures: %d" % len(frame.gestures())
      gesture = frame.gestures()[0]
      if (gesture.type == Leap.Gesture.TYPE_CIRCLE):
        circle = CircleGesture(gesture)
        # Determine clock direction using the angle between the pointable and the circle normal
        if circle.pointable.direction.angle_to(circle.normal) <= Leap.PI/4:
            clockwiseness = "clockwise"
        else:
            clockwiseness = "counterclockwise"

        # Calculate the angle swept since the last frame
        swept_angle = 0
        if circle.state != Leap.Gesture.STATE_START:
            previous_update = CircleGesture(controller.frame(1).gesture(circle.id))
            swept_angle =  (circle.progress - previous_update.progress) * 2 * Leap.PI

        rospy.loginfo("Circle id: %d, %s, progress: %f, radius: %f, angle: %f degrees, %s" % (
                gesture.id, self.state_string(gesture.state),
                circle.progress, circle.radius, swept_angle * Leap.RAD_TO_DEG, clockwiseness))

        if (circle.progress > 1.2):
          HandleCircleGesture(clockwiseness)
          self.last_gesture_time_ = time.time()

      if (gesture.type == Leap.Gesture.TYPE_SWIPE):
        swipe = SwipeGesture(gesture)
        rospy.loginfo("Swipe id: %d, position: %s, direction: %s, speed: %f, num_fingers: %s, classification: %s" % (
          gesture.id, swipe.position, swipe.direction, swipe.speed, swipe.pointable, self.classify_swipe(swipe)))
        HandleSwipeGesture(self.classify_swipe(swipe))
        self.last_gesture_time_ = time.time()


  def state_string(self, state):
    print "GestureListener state_string"
    if state == Leap.Gesture.STATE_START:
      return "STATE_START"

    if state == Leap.Gesture.STATE_UPDATE:
      return "STATE_UPDATE"

    if state == Leap.Gesture.STATE_STOP:
      return "STATE_STOP"

    if state == Leap.Gesture.STATE_INVALID:
      return "STATE_INVALID"

  def classify_swipe(self, swipe):
    direction = swipe.direction
    if (abs(direction.x) > 2. * abs(direction.y) and abs(direction.x) > 2. * abs(direction.z)):
      return "right" if direction.x > 0 else "left"
    elif (abs(direction.y) > 2. * abs(direction.x) and abs(direction.y) > 2. * abs(direction.z)):
      return "up" if direction.y > 0 else "down"      
    elif (abs(direction.z) > 2. * abs(direction.x) and abs(direction.z) > 2. * abs(direction.y)):
      return "forward" if direction.z < 0 else "backwards"
    else:
      return "unknown"

def main():
  global active_robot_name_, skill_server_control_service_
	# Initialize Main Program Vars
  skill_server_control_service_ = None
  active_robot_name_ = sys.argv[1] if len(sys.argv) > 1 else 'Wheatley'
  print "Arguments: %s, %d" % (sys.argv, len(sys.argv))
  rospy.init_node('LeapMotionServer')

  # Initialize Controller Service to core_skill_execution_server
  SetActiveRobot(active_robot_name_)

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
