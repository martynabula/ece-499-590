#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW   = 'robot-vid-chan'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 640
ny = 480

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
v = ach.Channel(ROBOT_CHAN_VIEW)
v.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0


print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    # Get Frame
    img = np.zeros((newx,newy,3), np.uint8)
    c_image = img.copy()
    vid = cv2.resize(c_image,(newx,newy))
    [status, framesize] = v.get(vid, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vid,(nx,ny))
        img = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl", img)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )


    [status, framesize] = t.get(tim, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        pass
        #print 'Sim Time = ', tim.sim[0]
    else:
        raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#--------[ Do not edit above ]------------------------
#-----------------------------------------------------
    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # img        = cv image in BGR format

    ts = tim.sim[0] #Get the time before any procecssing
    screen_center = 0

    if i == 0:  #Opens file to write error values
	centerError = open("centerError","w")
	i= i+1  #Ensures this if statement isn't entered again

    #Find blue object and its contours 
    img = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
    lower_blue = np.array([100,0,0],dtype=np.uint8)
    upper_blue = np.array([255,0,0],dtype=np.uint8)
    bcontours,hierarchy = cv2.findContours(cv2.inRange(img,lower_blue,upper_blue), 1, 2)
    
    #Get the center of the blue object
    if (bcontours != []):
	cnt = bcontours[0]
	M = cv2.moments(cnt)
	screen_center = int(M['m10']/M['m00'])      
	# if the center is to the right of the middle of the screen
	if (screen_center>=320):
	    # get the distance from the center of the object to the center of the screen
	    dist = screen_center-320
	    print dist  
	    # adjust the right and left wheel
	    ref.ref[0] = -dist/750; 
	    ref.ref[1] = dist/750; 
	    r.put(ref);          
	# if the center is to the left of teh middle of the screen
    	else:
            # adjust the wheels
	    dist = 320 - screen_center
	    print dist
	    ref.ref[0] = dist/750; 
	    ref.ref[1] = -dist/750;
	    r.put(ref);

    [status, framesize] = t.get(tim, wait=False, last=True)
    tn = tim.sim[0]
    
    #write the error values into a file for plotting 
    centerError.write('%s \n' %str(320 - screen_center))            
    if (.025 - (tn-ts))>=0:
        time.sleep(.025 - (tn-ts))

    k = cv2.waitKey(5)
    if k == 27:
	break
 
#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
if (tn-ts)>=.1:
	a=1 
