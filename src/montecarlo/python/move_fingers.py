#!/usr/bin/env python

import rospy
from kaist_msgs.msg import FingerMoveRequest
import time
import os
import sys

def get_keystroke():
	os.system("stty raw")
	r = sys.stdin.read(1)
	os.system("stty sane")
	return r

class MoveFingers():
	def __init__(self):
		self.pub = rospy.Publisher('/FingerMoveRequst', FingerMoveRequest, queue_size=5)
	
	def openFingers(self):
		request = FingerMoveRequest()
		request.requesterid = 0
		request.names.append("rf1")
		request.names.append("rf2")
		request.values.append(-10)
		request.values.append(-10)
		self.pub.publish(request)

	def openTrigger(self):
		request = FingerMoveRequest()
		request.requesterid = 0
		request.names.append("rf2")
		request.values.append(-100)
		self.pub.publish(request)
	
	def closeFingers(self):
		request = FingerMoveRequest()
		request.requesterid = 0
		request.names.append("rf1")
		request.names.append("rf2")
		request.values.append(10)
		request.values.append(10)
		self.pub.publish(request)

	def closeTrigger(self):
		request = FingerMoveRequest()
		request.requesterid = 0
		request.names.append("rf2")
		request.values.append(100)
		self.pub.publish(request)

	def stopFingers(self):
		request = FingerMoveRequest()
		request.requesterid = 0
		request.names.append("rf1")
		request.names.append("rf2")
		request.values.append(0)
		request.values.append(0)
		self.pub.publish(request)

if __name__ == '__main__':
	try:
		rospy.init_node('boxer_hands', anonymous=True)
		kaist_says = MoveFingers()

		key = 'a'
		while key != 'e':
			if key == 'o':
				kaist_says.openFingers()
				time.sleep(6.0)
				#kaist_says.openTrigger()
				#time.sleep(3)
				kaist_says.stopFingers()
			elif key == 'c':
				kaist_says.closeFingers()		
				time.sleep(4.0)
				#kaist_says.closeTrigger()
				#time.sleep(0.9)
				kaist_says.stopFingers()
					
			kaist_says.stopFingers()
			key = get_keystroke()

	except rospy.ROSInterruptException: pass



