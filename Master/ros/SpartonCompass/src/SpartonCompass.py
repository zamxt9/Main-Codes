#!/usr/bin/env python
# Software License Agreement (BSD License)

# Sparton Digital Compass ROS Driver for SP3003D/AHRS-8/GEDC-6
# Copyright (c) 2012, Cheng-Lung Lee, University of Detroit Mercy.
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
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
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
#
# Changelog
# 2012.12.13 Use Pos2D message, normalized to 0 ~ 2*PI
#


import roslib; roslib.load_manifest('SpartonCompass')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

import serial, string, math, time, calendar


#Check the NMEA sentence checksum. Return True if passes and False if failed
def check_checksum(nmea_sentence):
	if  ('$' in nmea_sentence) and ('*' in nmea_sentence) :
	    split_sentence = nmea_sentence.split('*')
	    transmitted_checksum = split_sentence[1].strip()
	    
	    #Remove the $ at the front
	    data_to_checksum = split_sentence[0][1:]
	    checksum = 0
	    for c in data_to_checksum:
		checksum ^= ord(c)

	    return ("%02X" % checksum)  == transmitted_checksum.upper()
        else:
            return 0

def wrapTo2PI(theta):
    '''Normalize an angle in radians to [0, 2*pi]
    '''
    return theta % (2.*math.pi)

def wrapToPI(theta):
    '''Normalize an angle in radians to [-pi, pi]
    '''
    return (wrapTo2PI(theta+math.pi) - math.pi)

if __name__ == '__main__':
    rospy.init_node('SpartonDigitalCompass')
    pub = rospy.Publisher('SpartonCompass', Pose2D)
    SpartonPose2D=Pose2D()
    SpartonPose2D.x=float(0.0)
    SpartonPose2D.y=float(0.0)
    #Init D_Compass port
    D_Compassport = rospy.get_param('~port','/dev/ttyUSB0')
    D_Compassrate = rospy.get_param('~baud',115200)
    #D_Compassrate = rospy.get_param('~baud',9600)
    D_Compassfreq = rospy.get_param('~frequency',10)
    #Digital compass heading offset in degree
    D_Compass_offset = rospy.get_param('~offset',0.)
    
    try:
        #talker()
        #ReadCompass()
	#Setup Compass serial port
        D_Compass = serial.Serial(port=D_Compassport, baudrate=D_Compassrate, timeout=1)
	# Set Digital Compass in Continous Mode with right Frequency
	# Send $ first with 10ms delay ( make it compatiable with SP3003D) then sned "xxHDT,PRT0.1<CR><LF>"
	time.sleep(0.1)
        D_Compass.write('$')
 	time.sleep(0.1)
        myStr="xxHDT,RPT=%4.3f" % (1.0/D_Compassfreq)
        D_Compass.write(myStr+'\r\n')
	rospy.loginfo('Send to Digital Compass Serial:'+myStr)
        #data = D_Compass.readline()
        #Read in D_Compass
        while not rospy.is_shutdown():
            #read D_Compass line  , The data example $HCHDT,295.5,T*2B
            #                                        [0]    [1]   
            data = D_Compass.readline()
            #rospy.loginfo("Received a sentence: %s" % data)

            if not check_checksum(data):
                rospy.logerr("Received a sentence with an invalid checksum. Sentence was: %s" % data)
                continue

            timeNow = rospy.get_rostime()
            fields = data.split(',')
            for i in fields:
                i = i.strip(',')

            try:
		    if 'HCHDT' in fields[0]:
	 		#convert heading (degrees) to theta (radious) , theta=(90-compassdir)/(360.)*(2*pi)
		       	#SpartonPose2D.theta = wrapToPI(((90.-float(fields[1]))/360.)*(2.*math.pi))
			#SpartonPose2D.x = float(fields[1]) #just for debug
		       	SpartonPose2D.theta = wrapToPI(math.radians(90.-float(fields[1])-D_Compass_offset))
        		pub.publish(SpartonPose2D)
		    else:
                        rospy.logerr("Received a sentence but not $HCHDT. Sentence was: %s" % data)

            except ValueError as e:
                rospy.logwarn("Value error, likely due to missing fields in the NMEA messages. No HCHDT header")

    except rospy.ROSInterruptException:
        D_Compass.write('$')
 	time.sleep(0.1)
        D_Compass.write('xxHDT\r\n')
	rospy.loginfo('Send to Digital Compass Serial:'+myStr)
        D_Compass.close() #Close D_Compass serial port
