#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
#import roslib; roslib.load_manifest('numpy_tutorials')
import rospy
#from std_msgs.msg import String
from std_msgs.msg import Int16
import serial
from array import array
import numpy as np
import struct as st

ser = serial.Serial('/dev/ttyACM0', 115200)
def talker():
    data_raw=[]	
    data1=[]
    pub = rospy.Publisher('chatter', Int16, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1000) # 10hz
    float_size=4
#    hello_float = Float32MultiArray()
#    hello_float.data = []
    while not rospy.is_shutdown():
	del data_raw[:]
        #hello_str = "hello world %s" % rospy.get_time()
#	data_raw.extend(ser.read(4))
#        for c in ser.read():
#		data_raw.append(c)
#		if c == '\n':
#		    print("Line: ",  data_raw)
#		    break
	data_raw = ser.readline()
#	data_raw = array('B', ''.join(data_raw)).tolist()
#	np.array(data_raw,dtype="int8")
	print "data_raw", data_raw
	if data_raw[18:19] :
            try:
		print "data", data_raw
                values = [float(j) for j in data_raw.split(",") if len(j) > 2]
                print "values", values
		if values[0]>30000:
			print "done"
		else :
			print "none"
            except ValueError:
                print "invalid string:"
####################################################################
		data1=data_raw[0:4]
		print "data1", data1
		sense = data_raw[3]+(data_raw[2]<<8)+(data_raw[1]<<16) + (data_raw[0]<<24)
		print "val", sense
#		#aa= bytearray(data1) 
#		data = array('B', data1).tostring()
#		print data
#		sensor1= st.unpack('<f', sense)
#	#	sensor1= data1.view(dtype=np.float32)
#		print "sensor1", sensor1
		if sense > 30000 :
			print "done"
		else : print "none"
################################################################
		data2=data_raw[4:8]
		print "data2", data2
		sense2 = data_raw[7]+(data_raw[6]<<8)+(data_raw[5]<<16) + (data_raw[4]<<24)
		print "val2", sense2
		if sense2 > 300000 :
			print "done2"
		else : print "none2"
########################################################
		data3=data_raw[9:12]
		print "data3", data3
		sense3 = data_raw[11]+(data_raw[10]<<8)+(data_raw[9]<<16) + (data_raw[8]<<24)
		print "val3", sense3
		if sense3 > 300000 :
			print "done3"
		else : print "none3"
############################################################
		data4=data_raw[13:16]
		print "data4", data4
		sense4 = data_raw[15]+(data_raw[14]<<8)+(data_raw[13]<<16) + (data_raw[12]<<24)
		print "val4", sense4
		if sense4 > 300000 :
			print "done4"
		else : print "none4"
		if sense4 > 300000 :
			print "done4"
		else : print "none4"
#################################################################
		data5=data_raw[17:20]
		print "data5", data5
		sense5 = data_raw[19]+(data_raw[18]<<8)+(data_raw[17]<<16) + (data_raw[16]<<24)
		print "val5", sense5
		if sense5 > 300000 :
			print "done5"
		else : print "none5"
#	data=data_raw[1:6*float_size]
#	print "data", data
#	datas=data.split(",")	
#	#print "data raw", data_raw[1], data_raw[2]
#	data1=datas[0:1]
#	print "data1", data1
#	type(data1)
	#data_raw.extend(ser.read(6*float_size))
	#hello_float=data1
#	print data_raw[1*float_size+1:2*float_size]
	#print hello_float.data[1:6]
        #rospy.loginfo(hello_float)
        #pub.publish(hello_float)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
