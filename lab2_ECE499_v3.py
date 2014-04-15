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
    if i == 0:
	Atime = open("Atime","w")
 	Etime = open("Etime","w")
	i= i+1
    ts = tim.sim[0]
    img = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
    lower_blue = np.array([100,0,0],dtype=np.uint8)
    upper_blue = np.array([255,0,0],dtype=np.uint8)
    lower_green = np.array([0,100,0],dtype=np.uint8)
    upper_green = np.array([0,255,0],dtype=np.uint8)
    lower_red = np.array([0,0,100],dtype=np.uint8)
    upper_red = np.array([0,0,255],dtype=np.uint8)
    mask = cv2.inRange(img,lower_red,upper_red)
    mask2 = cv2.inRange(img,lower_green,upper_green)
    mask3 = cv2.inRange(img,lower_blue,upper_blue)   

    rcontours,hierarchy = cv2.findContours(mask, 1, 2)
    gcontours,hierarchy = cv2.findContours(mask2, 1, 2)
    bcontours,hierarchy = cv2.findContours(mask3, 1, 2)
    x = 0;
    if (rcontours != []) and x == 0:
	cnt = rcontours[0]
	M = cv2.moments(cnt)
	cx = int(M['m10']/M['m00'])
	cy = int(M['m01']/M['m00'])
	if (cx>=313) and (cx<=328):
	    ref.ref[0] = 0;
	    ref.ref[1] = 0;
	    r.put(ref);
	    x = 1;
	    time.sleep(10)
        if (cx <=100):
	    x = 0;
    if (gcontours != []) and x == 0:
	cnt = gcontours[0]
	M = cv2.moments(cnt)
	cx = int(M['m10']/M['m00'])
	cy = int(M['m01']/M['m00'])
	if (cx>=313) and (cx<=328):
	    ref.ref[0] = 0;
	    ref.ref[1] = 0;
	    r.put(ref);
	    x = 1;
	    time.sleep(10)
   	if (cx <=100):
	    x = 0;
    if (bcontours != []) and x == 0:
	cnt = bcontours[0]
	M = cv2.moments(cnt)
	cx = int(M['m10']/M['m00'])
	cy = int(M['m01']/M['m00'])
	if (cx>=313) and (cx<=328):
	    ref.ref[0] = 0;
	    ref.ref[1] = 0;
	    r.put(ref);
	    x = 1;
	    time.sleep(10)
    	if (cx <=100):
	    x = 0
    ref.ref[0] = -0.5
    ref.ref[1] = 0.5
    # Sets reference to robot
    r.put(ref);
    [status, framesize] = t.get(tim, wait=False, last=True)
    tn = tim.sim[0]
    if (tn-ts)>=.1:
	a=1        
	Atime.write('%s \n' %str(tn))
        Etime.write("0")
    else:
        time.sleep(.05-(tn-ts))
        Atime.write('%s \n' %str(tn))
        Etime.write("%s \n" %str(tn-ts))

    k = cv2.waitKey(5)
    if k == 27:
	break
 
#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
