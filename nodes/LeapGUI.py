#!/usr/bin/env python
import os
import sys
import roslib; roslib.load_manifest("LeapMotion")
import sys
import rospy
import core_skill_execution_server
from core_skill_execution_server.srv import *
from core_skill_execution_server.msg import *
import time
import pygame
from pygame.locals import *

sys.path.insert(0, "./lib/")
import Leap
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture


active_robot_name_ = "GLaDOS"
skill_server_control_publisher_ = None
skill_server_status_request_ = None
root_gesturenode = None
current_gesturenode_ = None
failure_gesturenode_ = None
victory_node_ = None
highfive_node_ = None
left_handover = None
num_consecutive_failures_ = 0
action_history_ = []


if not pygame.font: print 'Fonts disabled'
if not pygame.mixer: print 'Sounds disabled'


class GestureNode:
  gesture_types = ["swipe_left", "swipe_right", "swipe_forward", "swipe_backward", "swipe_up", "swipe_down", "rotate_ccw", "rotate_cw"]

  def __init__(self, primitive_action):
    self.children = {}
    self.sound_path = '../sounds/success.mp3'
    #for gesture_type in self.gesture_types:
    #  self.children[gesture_type] = None
    self.primitive_action = primitive_action

  def __str__(self):
    s = ""
    for gesture, node in self.children.iteritems():
      if node == None: continue
      s = "%s [%s: %s]" % (s, gesture, node.primitive_action)
    return s

  def AddLinkedNode(self, gesture_type, node):
    if not isinstance(node, GestureNode): return False
    #if not gesture_type in self.children: return False
    self.children[gesture_type] = node
    return True

  def PlayAction(self):
    print "--Playing primitive action %s" % self.primitive_action
    ExecuteSkill(self.primitive_action)

  def PlaySound(self):
    pygame.mixer.music.load(self.sound_path)
    pygame.mixer.music.play(1)
    print "--Playing sound %s" % self.sound_path

  def ProcessGesture(self, gesture_type):
    if (not gesture_type in self.children) or (self.children[gesture_type] == None): 
      return None

    return self.children[gesture_type]


class GuiMain:

  def __init__(self, width=800, height=250):
    pygame.init()
    self.width = width
    self.height = height
    self.screen = pygame.display.set_mode((self.width,self.height))
    self.background = pygame.Surface(self.screen.get_size())
    self.background = self.background.convert()
    self.background.fill((0,0,0))

  def MainLoop(self):
    while not rospy.is_shutdown():
      for event in pygame.event.get():
        if event.type == pygame.QUIT:
          sys.exit()
        if event.type == pygame.KEYDOWN:
          if event.key == pygame.K_q:
            sys.exit()
          elif event.key == pygame.K_r:
            ResetGameState()
          elif event.key == pygame.K_d:
            PlayVictory()
          elif event.key == pygame.K_h:
            PlayHighFive()
          elif event.key == pygame.K_RIGHT:
            ProcessGesture("swipe_right")
          elif event.key == pygame.K_DOWN:
            ProcessGesture("swipe_down")
          elif event.key == pygame.K_DELETE:
            ProcessGesture("circle_ccw")
          elif event.key == pygame.K_PAGEDOWN:
            ProcessGesture("circle_cw")


          #Reset key/script
      self.screen.blit(self.background,(0,0))
      self.RenderActiveRobot()
      self.RenderActiveGesture()
      self.RenderGestureOptions()
      self.RenderControls()
      EvaluateGameLogic()
      pygame.display.flip()

  def RenderActiveRobot(self):
    global active_robot_name_
    font = pygame.font.SysFont("monospace", 20, False, False)
    text = font.render("Active Robot: %s" % active_robot_name_, False, (255, 255, 255))
    textpos = text.get_rect(centerx = self.width/2)
    self.screen.blit(text,textpos)

  def RenderActiveGesture(self):
    global current_gesturenode_
    font = pygame.font.SysFont("monospace", 20, False, False)
    text = font.render("Active Primitive: %s" % current_gesturenode_.primitive_action, False, (255, 255, 255))
    textpos = text.get_rect(centerx = self.width/2, centery = 50)
    self.screen.blit(text,textpos)

  def RenderGestureOptions(self):
    global current_gesturenode_
    font = pygame.font.SysFont("monospace", 16, False, False)
    text = font.render("Gestures: %s" % str(current_gesturenode_), False, (255, 255, 255))
    textpos = text.get_rect(centerx = self.width/2, centery = 100)
    self.screen.blit(text,textpos)

  def RenderControls(self):
    font = pygame.font.SysFont("monospace", 16, False, False)
    text = font.render("Key Commands:      [R]eset     [Q]uit    [D]ance     [H]ighfive", False, (0, 255, 0))
    textpos = text.get_rect(centerx = self.width/2, centery = self.height - 40)
    self.screen.blit(text,textpos)




class GestureListener(Leap.Listener):
  last_gesture_time_ = 0

  def on_init(self, controller):
    print "GestureListener on_init"

  def on_connect(self, controller):
    print "GestureListener on_connect"
    controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE)
    #controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP)
    #controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP)
    controller.enable_gesture(Leap.Gesture.TYPE_SWIPE)

  def on_disconnect(self, controller):
    print "GestureListener on_disconnect"

  def on_frame(self, controller):
    frame = controller.frame()

    #print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
    #      frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

    if (len(frame.gestures()) > 0 and time.time() - GestureListener.last_gesture_time_ > 1):
      print "Number of Gestures: %d.   last_gesture_time_: %d       time: %d" % (len(frame.gestures()), GestureListener.last_gesture_time_, time.time())
      gesture = frame.gestures()[0]
      if (gesture.type == Leap.Gesture.TYPE_CIRCLE):
        circle = CircleGesture(gesture)
        # Determine clock direction using the angle between the pointable and the circle normal
        if circle.pointable.direction.angle_to(circle.normal) <= Leap.PI/4:
            clockwiseness = "cw"
        else:
            clockwiseness = "ccw"

        # Calculate the angle swept since the last frame
        swept_angle = 0
        if circle.state != Leap.Gesture.STATE_START:
            previous_update = CircleGesture(controller.frame(1).gesture(circle.id))
            swept_angle =  (circle.progress - previous_update.progress) * 2 * Leap.PI

        rospy.loginfo("Circle id: %d, %s, progress: %f, radius: %f, angle: %f degrees, %s" % (
                gesture.id, self.state_string(gesture.state),
                circle.progress, circle.radius, swept_angle * Leap.RAD_TO_DEG, clockwiseness))

        if (circle.progress > 1.2): # and circle.state == Leap.Gesture.STATE_STOP):
          GestureListener.last_gesture_time_ = time.time()
          HandleCircleGesture(clockwiseness)

      if (gesture.type == Leap.Gesture.TYPE_SWIPE):
        swipe = SwipeGesture(gesture)
        rospy.loginfo("Swipe id: %d, position: %s, direction: %s, speed: %f, num_fingers: %s, classification: %s" % (
          gesture.id, swipe.position, swipe.direction, swipe.speed, swipe.pointable, self.classify_swipe(swipe)))
        if (self.classify_swipe(swipe) != "unknown"):
          GestureListener.last_gesture_time_ = time.time()
          HandleSwipeGesture(self.classify_swipe(swipe))


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
      return "forward" if direction.z < 0 else "backward"
    else:
      return "unknown"




# GAME CONTROL CODE

def ProcessGesture(gesture_type):
  global current_gesturenode_, failure_gesturenode_, num_consecutive_failures_, root_gesturenode, action_history_

  next_gesturenode = None
  next_gesturenode = current_gesturenode_.ProcessGesture(gesture_type)

  if next_gesturenode == None:
    # Invalid transition given
    num_consecutive_failures_ += 1
    if (num_consecutive_failures_ >= 50):
      print "Following invalid transition from %s" % (current_gesturenode_.primitive_action)
      failure_gesturenode_.PlaySound()
      failure_gesturenode_.PlayAction()
      ResetGameState()
    return False
  else:
    # Valid transition given
    print "Following valid transition from %s to %s" % (current_gesturenode_.primitive_action, next_gesturenode.primitive_action)
    num_consecutive_failures_ = 0
    next_gesturenode.PlaySound()
    next_gesturenode.PlayAction()
    root_gesturenode.PlaySound() # Play the "ready" noise
    current_gesturenode_ = next_gesturenode
    action_history_.append(current_gesturenode_);
    return True

def HandleSwipeGesture(swipe_classification):
  ProcessGesture("swipe_" + swipe_classification)

def HandleCircleGesture(circle_classification):
  ProcessGesture("rotate_"+circle_classification)

def EvaluateGameLogic():
  global current_gesturenode_
  if (current_gesturenode_ == None):
    print "Current gesturenode is None! Error."
    ResetGameState()
  if len(current_gesturenode_.children) == 0:
    PlayWholePath()
    PlayVictory()


def PlayWholePath():
  global action_history_, root_gesturenode, current_gesturenode_, left_handover
  print "Primitive Action of Current at PlayWholePath: %s" % current_gesturenode_.primitive_action
  if (current_gesturenode_.primitive_action == "LeftPickupHandover"):
    pygame.mixer.music.load("../sounds/partyrock.wav")
  else:
    pygame.mixer.music.load("../sounds/finished_sequence.wav")

  pygame.mixer.music.play(1)
  root_gesturenode.PlayAction()
  for gesturenode in action_history_:
    gesturenode.PlayAction()

def PlayVictory():
  global victory_node_
  print "==Victory condition=="
  victory_node_.PlaySound()
  victory_node_.PlayAction()
  ResetGameState()    

def PlayHighFive():
  global highfive_node_
  print "==Highfive condition=="
  highfive_node_.PlaySound()
  highfive_node_.PlayAction()
  ResetGameState()    

def ResetGameState():
  global current_gesturenode_, root_gesturenode, action_history_, num_consecutive_failures_
  print "Reset game state"
  action_history_ = []
  num_consecutive_failures_ = 0
  current_gesturenode_ = root_gesturenode
  root_gesturenode.PlaySound()
  root_gesturenode.PlayAction()


# ROBOT CONTROL CODE

def SetActiveRobot(robot_name):
  global active_robot_name_, skill_server_control_service_
  active_robot_name_ = robot_name
  rospy.loginfo("LeapServer setting active robot to %s..." % active_robot_name_)
  InitializeSkillServerPublisher()
  rospy.loginfo("...LeapServer set active robot to %s" % active_robot_name_)

def InitializeSkillServerPublisher():
  global active_robot_name_, skill_server_control_publisher_, skill_server_status_request_
  skill_server_topic_name = "/"+active_robot_name_+"/skillserver_control_command"
  rospy.loginfo("Setting up publisher for core_skill_execution_server (Topic: %s)" % skill_server_topic_name)
  skill_server_control_publisher_ = rospy.Publisher(skill_server_topic_name, core_skill_execution_server.msg.SkillserverCommand)

  # Loop until get status returns something ok
  skill_server_status_service_name = "/"+active_robot_name_+"/skillserver_statusrequest"
  rospy.wait_for_service(skill_server_status_service_name)
  skill_server_status_request_ = rospy.ServiceProxy(skill_server_status_service_name, StatusRequest)
  rospy.loginfo("Connected")


def ExecuteSkill(skill_name):
  global active_robot_name_, skill_server_control_publisher_
  print "Executing %s on robot: %s" % (skill_name, active_robot_name_)

  msg = SkillserverCommand()
  msg.command_type = msg.COMMAND_LOAD_SKILL
  msg.command_string_args.append(skill_name)
  skill_server_control_publisher_.publish(msg)

  srv = StatusRequestRequest()
  srv.request_type = StatusRequestRequest.REQUEST_ACTIVE_SKILL
  resp = skill_server_status_request_(srv)
  if (resp.string_values[0] != skill_name): #string_keys[0] is ActiveSkill for this service call
    rospy.logerr("Couldn't load skill %s on robot %s" % (skill_name, active_robot_name_))
    return False

  msg = SkillserverCommand()
  msg.command_type = msg.COMMAND_EXECUTE_SKILL
  rospy.loginfo("Beginning Skill Execution...")
  skill_server_control_publisher_.publish(msg)

  rospy.loginfo("...Activated Skill Execution")

  # Make this a blocking function
  while True:
    rospy.sleep(0.5)
    srv = StatusRequestRequest()
    srv.request_type = StatusRequestRequest.REQUEST_EXECUTE_STATUS
    resp = skill_server_status_request_(srv)
    print "%s: %s." % (resp.bool_keys[0], resp.bool_values[0])
    if (resp.bool_values[0] == False):
      break

  rospy.loginfo("...Finished Skill Execution")

  return True

  
def main():
  global skill_server_control_service_, active_robot_name_, current_gesturenode_, failure_gesturenode_, root_gesturenode, highfive_node_, victory_node_, left_handover
  # Initialize Main Program Vars
  skill_server_control_service_ = None
  active_robot_name_ = sys.argv[1] if len(sys.argv) > 1 else 'GLaDOS'
  print "Arguments: %s, %d" % (sys.argv, len(sys.argv))
  rospy.init_node('LeapMotionServer')

  # Initialize Controller Service to core_skill_execution_server
  SetActiveRobot(active_robot_name_)

  # Initialize connection to LeapMotion
  leap_listener_ = GestureListener()
  leap_controller_ = Leap.Controller()
  leap_controller_.add_listener(leap_listener_)

  
  # Set up gesture network
  root_gesturenode = GestureNode("ReadyPosition")
  root_gesturenode.sound_path = "../sounds/ready.wav"

  failure_gesturenode_ = GestureNode("FailureAction")
  failure_gesturenode_.sound_path = "../sounds/failure.wav"

  victory_node_ = GestureNode("VictoryDance")
  victory_node_.sound_path = "../sounds/victory.wav"

  highfive_node_ = GestureNode("HighFive")
  highfive_node_.sound_path = "../sounds/highfive.wav"

  # Sequence 1:
  # Readyposition, StandUp, StandupFaceRight, FaceRightBendOver, FaceRightBendOverPickUp, RightPickupHandover
  stand_up = GestureNode("StandUp")
  face_right = GestureNode("StandupFaceRight")
  right_bend_over = GestureNode("FaceRightBendOver")
  right_pickup = GestureNode("FaceRightBendOverPickUp")
  right_handover = GestureNode("RightPickupHandover") 

  root_gesturenode.AddLinkedNode("swipe_up",stand_up)
  stand_up.AddLinkedNode("rotate_ccw", face_right)
  face_right.AddLinkedNode("swipe_right", right_bend_over)
  right_bend_over.AddLinkedNode("swipe_down", right_pickup)
  right_pickup.AddLinkedNode("swipe_backward", right_handover)


  # Sequence 2:
  # Readyposition, BaseRotateCCW, LeftReadyPickup, LeftPickup, LeftPickupHandover
  base_rotate_ccw = GestureNode("BaseRotateCCW")
  left_ready_pickup = GestureNode("LeftReadyPickup")
  left_pickup = GestureNode("LeftPickup")
  left_handover = GestureNode("LeftPickupHandover")
  #left_handover.sound_path = "../sounds/partyrock.wav"

  root_gesturenode.AddLinkedNode("rotate_ccw", base_rotate_ccw)
  base_rotate_ccw.AddLinkedNode("swipe_left", left_ready_pickup)
  left_ready_pickup.AddLinkedNode("swipe_down", left_pickup)
  left_pickup.AddLinkedNode("swipe_backward", left_handover)

  # Sequence 3:
  # Readyposition, StandUp, StandUpFaceForward, ScrewIn, StandUpFaceForwardLow, ScrewIn
  face_forward = GestureNode("StandUpFaceForward")
  screw_in_fwd = GestureNode("ScrewIn")
  face_forward_low = GestureNode("StandupFaceForwardLow")
  screw_in_final = GestureNode("ScrewIn")

  stand_up.AddLinkedNode("swipe_backward", face_forward)
  face_forward.AddLinkedNode("rotate_cw", screw_in_fwd)
  screw_in_fwd.AddLinkedNode("swipe_down", face_forward_low)
  face_forward_low.AddLinkedNode("rotate_cw", screw_in_final)

  # Sequence 4:
  # Readyposition, StandUp, StandUpFaceBack, ScrewIn, StandUpFaceBackLow, ScrewIn
  face_back = GestureNode("StandUpFaceBack")
  screw_in_back = GestureNode("ScrewIn")
  face_back_low = GestureNode("StandUpFaceBackLow")

  stand_up.AddLinkedNode("swipe_forward", face_back)
  face_back.AddLinkedNode("rotate_cw", screw_in_back)
  screw_in_back.AddLinkedNode("swipe_down", face_back_low)
  face_back_low.AddLinkedNode("rotate_cw", screw_in_final)

  current_gesturenode_ = root_gesturenode

  # Start up GUI
  MainWindow = GuiMain()
  MainWindow.MainLoop()

  leap_controller_.remove_listener(leap_listener_)



if __name__ == "__main__":
  main()
