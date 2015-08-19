#!/usr/bin/env python
# .. -*- coding: utf-8 -*-
#
# **************************************************
# drone_controller.py - Basic drone controller class
# **************************************************
# This code began based on the basic drone controller
# class for the tutorial `Up and flying with the AR.Drone
# and ROS | Getting Started`
# <https://github.com/mikehamer/ardrone_tutorials_getting_started>`_
#
# This class implements basic control functionality which
# we will be using in future tutorials. It can command
# takeoff/landing/emergency as well as drone movement.
# It also tracks the drone state based on navdata feedback
#
# Imports
# =======
# Imports are listed in the order prescribed by `PEP 8
# <http://www.python.org/dev/peps/pep-0008/#imports>`_.
#
# Library imports
# ---------------
# None needed.
#
# Third-party imports
# -------------------
# Import the ROS libraries.
import rospy
# Import the messages we're interested in sending and
# receiving:
#
# For sending movement commands to the drone.
from geometry_msgs.msg import Twist
# For sending land/takeoff/emergency commands.
from std_msgs.msg import Empty
# For receiving navdata feedback.
from ardrone_autonomy.msg import Navdata
# For calling services wihch take no parameters: toggle
# camera and flat trim.
from std_srvs.srv import Empty as EmptyServiceType
# For calling services which take parameters.
from ardrone_autonomy.srv import CamSelect, \
  FlightAnim, LedAnim, RecordEnable
#
# Local imports
# -------------
# An enumeration of Drone Status.
from drone_status import DroneStatus
#
#
# BasicDroneController
# ====================
# This class provide a Pythonic interface to the drone.
class BasicDroneController(object):
    # Some Constants.
    COMMAND_PERIOD = 100 #ms

    def __init__(self):
        # Holds the current drone status.
        self.status = -1

        # Proivde services
        # ----------------
        # For now, the ``rospy.wait_for_service`` calls are
        # disabled -- that way, this program will still run
        # even if the drone isn't connected.
        #
        # `Toggle camera
        # <http://ardrone-autonomy.readthedocs.org/en/latest/services.html#toggle-camera>`_
        toggle_camera = '/ardrone/togglecam'
        #rospy.wait_for_service(toggle_camera)
        self.ToggleCamera = rospy.ServiceProxy(
          toggle_camera, EmptyServiceType)
        # Set camera channel (see link above).
        set_camera_channel = '/ardrone/setcamchannel'
        #rospy.wait_for_service(set_camera_channel)
        self.SetCamera = rospy.ServiceProxy(
          set_camera_channel, CamSelect)
        # `LED animations
        # <http://ardrone-autonomy.readthedocs.org/en/latest/services.html#led-animations>`_
        led_animations = '/ardrone/setledanimation'
        #rospy.wait_for_service(led_animations)
        self.SetLedAnimation = rospy.ServiceProxy(
          led_animations, LedAnim)
        # 'Flight animations
        # <http://ardrone-autonomy.readthedocs.org/en/latest/services.html#flight-animations>`_
        # Be careful with these!
        flight_animations = '/ardrone/setflightanimation'
        #rospy.wait_for_service(flight_animations)
        self.SetFlightAnimation = rospy.ServiceProxy(
          flight_animations, FlightAnim)
        # `Flat trim
        # <http://ardrone-autonomy.readthedocs.org/en/latest/services.html#flat-trim>`_
        flat_trim = '/ardrone/flattrim'
        #rospy.wait_for_service(flat_trim)
        self.SetFlatTrim = rospy.ServiceProxy(flat_trim,
          EmptyServiceType)
        # `Record to USB stick
        # <http://ardrone-autonomy.readthedocs.org/en/latest/services.html#record-to-usb-stick>`_
        record_usb = '/ardrone/setrecord'
        #rospy.wait_for_service(record_usb)
        self.RecordUsb = rospy.ServiceProxy(record_usb,
          RecordEnable)

        # Takeoff, land, and reset
        # ------------------------
        # Allow the controller to publish to the
        # ``/ardrone/takeoff``, ``land`` and ``reset``
        # topics.
        self.pubLand = rospy.Publisher('/ardrone/land',
          Empty, queue_size=10)
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',
          Empty, queue_size=10)
        self.pubReset = rospy.Publisher('/ardrone/reset',
          Empty, queue_size=10)

        # Velocity
        # --------
        # Allow the controller to publish to the
        # ``/cmd_vel`` topic and thus control the drone.
        self.pubCommand = rospy.Publisher('/cmd_vel',
          Twist, queue_size=10)

        # Subscribe to the ``/ardrone/navdata`` topic, of
        # message type navdata, and call
        # ``self.ReceiveNavdata`` when a message is
        # received.
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',
          Navdata, self._ReceiveNavdata)

        # Set up regular publishing of control packets.
        self.command = Twist()
        self.commandTimer = rospy.Timer(rospy.Duration(
          self.COMMAND_PERIOD/1000.0), self._SendCommand)

        # Shutdown
        # --------
        # Land the drone when we shut down.
        rospy.on_shutdown(self.SendLand)

    # Send a takeoff message to the ardrone driver.
    def SendTakeoff(self):
        # Note we only send a takeoff message if the drone
        # is landed - an unexpected takeoff is not good!
        if self.status == DroneStatus.Landed:
            self.pubTakeoff.publish(Empty())

    # Send a landing message to the ardrone driver.
    def SendLand(self):
        # Note we send this in all states; landing can do no
        # harm.
        self.pubLand.publish(Empty())

    # Send an emergency (or reset) message to the
    # ardrone driver.
    def SendEmergency(self):
        self.pubReset.publish(Empty())

    # Define the flight command which will be sent to the
    # drone. All the commands accept values between -1 and
    # 1. A value of 0 commands no motion.
    def SetCommand(self,
      # Specify the left/right angle, causing the drone to
      # move sideways.
      roll=0,
      # Specify the front/back angle, causing the drone to
      # move forwards or backwards.
      pitch=0,
      # Specify the velocity at which the drone spins about
      # its center.
      yaw_velocity=0,
      # Specify the ascent/descent rate for the drone.
      z_velocity=0):
        self.command.linear.x  = pitch
        self.command.linear.y  = roll
        self.command.linear.z  = z_velocity
        self.command.angular.z = yaw_velocity

    # Command the drone to hover.
    def hover(self):
        self.SetCommand(0, 0, 0, 0)

    # Internal function -- do not call outside this class.
    #
    # The drone needs a flight command send to it
    # periodically. This method is invoked by a
    # continuously-running timer, which re-sends the last
    # flight command specified by ``SetCommand``.
    def _SendCommand(self, event):
        # The previously set command is then sent out
        # periodically if the drone is flying.
        if (self.status == DroneStatus.Flying or
            self.status == DroneStatus.GotoHover or
            self.status == DroneStatus.Hovering):
            self.pubCommand.publish(self.command)

    # Internal function -- do not call outside this class.
    #
    # This is invoked when the drone reports its navdata.
    # Save it for later use.
    def _ReceiveNavdata(self, navdata):
        # Although there is a lot of data in this packet,
        # we're only interested in the state at the moment.
        self.status = navdata.state


