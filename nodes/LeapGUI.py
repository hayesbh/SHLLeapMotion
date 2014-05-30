#!/usr/bin/env python
import os
import sys
import pygame
from pygame.locals import *

if not pygame.font: print 'Fonts disabled'
if not pygame.mixer: print 'Sounds disabled'

active_robot_ = "Wheatley"
current_gesturenode_ = None
failure_gesturenode_ = None

class GestureNode:
  gesture_types = ["swipe_left", "swipe_right", "swipe_forward", "swipe_backward", "swipe_up", "swipe_down", "circle_ccw", "circle_cw"]

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
    pass
    #play primitive_action    

  def PlaySound(self):
    pygame.mixer.music.load(self.sound_path)
    pygame.mixer.music.play(1)
    print "--Playing sound %s" % self.sound_path

  def ProcessGesture(self, gesture_type):
    if (not gesture_type in self.children) or (self.children[gesture_type] == None): 
      return None

    return self.children[gesture_type]


class GuiMain:

  def __init__(self, width=800, height=600):
    pygame.init()
    self.width = width
    self.height = height
    self.screen = pygame.display.set_mode((self.width,self.height))
    self.background = pygame.Surface(self.screen.get_size())
    self.background = self.background.convert()
    self.background.fill((0,0,0))

  def MainLoop(self):
    global active_robot_, current_gesturenode_
    while 1:
      for event in pygame.event.get():
        if event.type == pygame.QUIT:
          sys.exit()
        if event.type == pygame.KEYDOWN:
          if event.key == pygame.K_q:
            sys.exit()
          elif event.key == pygame.K_r:
            self.ResetGameState()
          else:
            self.ProcessGesture(event)

          #Reset key/script
      self.screen.blit(self.background,(0,0))
      self.RenderActiveRobot()
      self.RenderActiveGesture()
      self.RenderGestureOptions()
      self.EvaluateGameLogic()
      pygame.display.flip()

  def ProcessGesture(self, event):
    global current_gesturenode_, failure_gesturenode_

    next_gesturenode = None
    if event.key == pygame.K_RIGHT:
      next_gesturenode = current_gesturenode_.ProcessGesture("swipe_right")
    if event.key == pygame.K_DOWN:
      next_gesturenode = current_gesturenode_.ProcessGesture("swipe_down")
    if event.key == pygame.K_DELETE:
      next_gesturenode = current_gesturenode_.ProcessGesture("circle_ccw")
    if event.key == pygame.K_PAGEDOWN:
      next_gesturenode = current_gesturenode_.ProcessGesture("circle_cw")

    if next_gesturenode == None:
      # Invalid transition given
      print "Following invalid transition from %s" % (current_gesturenode_.primitive_action)
      failure_gesturenode_.PlaySound()
      failure_gesturenode_.PlayAction()
      self.ResetGameState()
    else:
      # Valid transition given
      print "Following valid transition from %s to %s" % (current_gesturenode_.primitive_action, next_gesturenode.primitive_action)
      next_gesturenode.PlaySound()
      next_gesturenode.PlayAction()
      current_gesturenode_ = next_gesturenode


  def EvaluateGameLogic(self):
    global current_gesturenode_, victory_node
    if len(current_gesturenode_.children) == 0:
      print "Resetting Game Logic -- End/win condition found"
      victory_node.PlaySound()
      victory_node.PlayAction()
      self.ResetGameState()

  def ResetGameState(self):
    global current_gesturenode_,root_gesturenode
    print "Reset game state"
    current_gesturenode_ = root_gesturenode
    root_gesturenode.PlaySound()
    root_gesturenode.PlayAction()

  def RenderActiveRobot(self):
    global active_robot_
    font = pygame.font.SysFont("monospace", 20, False, False)
    text = font.render("Active Robot: %s" % active_robot_, False, (255, 255, 255))
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


if __name__ == "__main__":
  root_gesturenode = GestureNode("ReadyPosition")

  failure_gesturenode_ = GestureNode("FailureAction")
  failure_gesturenode_.sound_path = "../sounds/failure.wav"

  victory_node = GestureNode("VictoryDance")
  victory_node.sound_path = "../sounds/victory.mp3"

  #root_gesturenode.sound_path = "../sounds/startup.mp3"

  # Sequence 1:  Gesture: SwR, SwD, CCW, CCW
  #               Action: SA1, SA2, SA3, SA4
  sequence_1_1 = GestureNode("SA1")
  sequence_1_2 = GestureNode("SA2")
  sequence_1_3 = GestureNode("SA3")
  sequence_1_4 = GestureNode("SA4")


  root_gesturenode.AddLinkedNode("swipe_right", sequence_1_1)
  sequence_1_1.AddLinkedNode("swipe_down", sequence_1_2)
  sequence_1_2.AddLinkedNode("circle_ccw", sequence_1_3)
  sequence_1_3.AddLinkedNode("circle_ccw", sequence_1_4)

  current_gesturenode_ = root_gesturenode
  MainWindow = GuiMain()
  MainWindow.MainLoop()