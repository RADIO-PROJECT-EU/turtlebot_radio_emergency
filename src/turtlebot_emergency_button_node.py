#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

# Import custom message data.
from kobuki_msgs.msg import SensorState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class TurtlebotEmergencyButton(object):
    
    def __init__(self):
        
        # Publishers
        # This publisher sends zero velocity linear and angular to the command_velocity topic
        self.twist_publisher = rospy.Publisher("/cmd_vel_mux/input/emergency_button", Twist, queue_size=10)
        # This publish the state of the emergency button
        self.emergency_state_publisher = rospy.Publisher("~state", Bool, queue_size=10)
        # Subscribers
        # This subscribe to the analog/digital input/output topic
        self.sensor_state_subscriber = rospy.Subscriber("/mobile_base/sensors/core", SensorState, self.SensorStateCallback, queue_size = 10)
        
        self.msg_twist = Twist()
        self.msg_state = Bool()

    def SensorStateCallback(self, data):
        #rospy.loginfo('TurtlebotEmergency:SensorStateCallback:: Received a message! ')
        input = data.analog_input[1]
        self.msg_twist.linear.x = 0.0
        self.msg_twist.linear.y = 0.0
        self.msg_twist.linear.z = 0.0
        self.msg_twist.angular.x = 0.0
        self.msg_twist.angular.y = 0.0
        self.msg_twist.angular.z = 0.0
        
        if input > 3500:
            #rospy.loginfo('TurtlebotEmergency:SensorStateCallback:: Emergency Button Pressed!! ')
            self.twist_publisher.publish(self.msg_twist)
            self.msg_state = True
        else:
            #rospy.loginfo('TurtlebotEmergency:SensorStateCallback:: Emergency Button Released!! ')
            self.msg_state = False
            
        self.emergency_state_publisher.publish(self.msg_state)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

def main():

    rospy.init_node("emergency_stop_button")
    _name = rospy.get_name().replace('/','')
    node = TurtlebotEmergencyButton()
    rospy.loginfo('%s: starting'%(_name))
    node.run()

if __name__ == "__main__":
    main()
