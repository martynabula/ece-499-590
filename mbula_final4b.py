#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro
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

import hubo_ach as ha
import ach
import sys
import time
from ctypes import *
import numpy as np

for args in sys.argv:
    variable = args

steplength = float(variable)
count = 0
finish = 5/steplength

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
#s.flush()
#r.flush()

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=False)

i = 0
while i < 10:
    ref.ref[ha.RSR] = ref.ref[ha.RSR]-.01
    ref.ref[ha.LSR] = ref.ref[ha.LSR]+.01
    i = i+.5
    r.put(ref)
    time.sleep(.01)

while finish >0:
	while count == 0:
	    i = 0		#ONE TIME ONLY
	    while i < .5: 	#GET READY  
		ref.ref[ha.RAP] = -i
		ref.ref[ha.LAP] = -i
		ref.ref[ha.LKN] = 2*i
		ref.ref[ha.RKN] = 2*i
		ref.ref[ha.RHP] = -i
		ref.ref[ha.LHP] = -i
		r.put(ref)
		i = i+ .05
		time.sleep(.5)
	    print 'ready'
	    i=0		#ONE TIME ONLY
	    while i < .15: 	#LEAN
		ref.ref[ha.RAR] = i
		ref.ref[ha.LAR] = i
		ref.ref[ha.RHR] = -i
		ref.ref[ha.LHR] = -i
		r.put(ref)
		i = i+ .01
		time.sleep(.5)
		
	    print 'lean'

	    i = .5
	    while i <.75: #RIGHT LEG UP
		ref.ref[ha.RAP] = -i
		ref.ref[ha.RKN] = 2*i
		ref.ref[ha.RHP] = -i
		r.put(ref)
		i = i+.05
		time.sleep(.5)

	    RAPref = ref.ref[ha.RAP]  #GET STEADY LEG REFERENCE POSITIONS
	    RKNref = ref.ref[ha.RKN]
	    RHPref = ref.ref[ha.RHP]
	    LAPref = ref.ref[ha.LAP]  
	    LKNref = ref.ref[ha.LKN] 
	    LHPref = ref.ref[ha.LHP]  
	    RARref = ref.ref[ha.RAR]
	    LARref = ref.ref[ha.LAR]
	    RHRref = ref.ref[ha.RHR]
	    LHRref = ref.ref[ha.LHR]

	    count = 1 #LEAVE INITIALIZATION LOOP

##############################################################################

	while count == 1:

	    l = .34003*np.cos(ref.ref[ha.RHP])+.34038*np.cos(ref.ref[ha.RHP]+ref.ref[ha.RKN])+.11497
	    thetaincrease = np.arcsin(steplength/l)

	    i = 0
	    while i<thetaincrease:	#FIND STEP DISTANCE
		ref.ref[ha.RHP] = ref.ref[ha.RHP]-.005
		ref.ref[ha.RAP] = ref.ref[ha.RAP]+.005
		i = i + .005
		r.put(ref)
		time.sleep(.05)

	    l1 = .34003*np.cos(ref.ref[ha.LHP])+.34038*np.cos(ref.ref[ha.LHP]+ref.ref[ha.LKN])+.11497
	    l2 = .34003*np.cos(ref.ref[ha.RHP])+.34038*np.cos(ref.ref[ha.RHP]+ref.ref[ha.RKN])+.11497
	    

	    while l1 > l2: 	#STEP DOWN
		ref.ref[ha.LAP] = ref.ref[ha.LAP]-.00005
		ref.ref[ha.LKN] = ref.ref[ha.LKN]+.0001
		ref.ref[ha.LHP] = ref.ref[ha.LHP]-.00005
		r.put(ref)
		time.sleep(.00005)
		l1 = .34003*np.cos(ref.ref[ha.LHP])+.34038*np.cos(ref.ref[ha.LHP]+ref.ref[ha.LKN])+.11497
		l2 = .34003*np.cos(ref.ref[ha.RHP])+.34038*np.cos(ref.ref[ha.RHP]+ref.ref[ha.RKN])+.11497
		time.sleep(.0005)

	    print "Stepped Down"

	    p = .34003*np.cos(ref.ref[ha.LHP])+.34038*np.cos(ref.ref[ha.LHP]+ref.ref[ha.LKN])+.11497
	    l = np.sqrt((p)*(p) + (steplength)*(steplength))
	    Checkr = ref.ref[ha.RAR]
	    Checkhp = ref.ref[ha.LHP]
	    Checkap = ref.ref[ha.LAP]
	    Checkkn = .05*(l-p)
	    HPrate = np.abs(.05*(ref.ref[ha.LHP]-ref.ref[ha.RHP]))
	    Rrate = np.abs(.1*Checkr)
	    #SHIFT WEIGHT

	    while Checkr >-ref.ref[ha.RAR] or ref.ref[ha.RHP] < Checkhp or ref.ref[ha.RAP] < Checkap or p<l:
		if ref.ref[ha.RAP] > Checkap:
		    ref.ref[ha.LAP] = -(ref.ref[ha.LHP]+ref.ref[ha.LKN])
		    ref.ref[ha.RAP] = ref.ref[ha.RAP] - HPrate
		if ref.ref[ha.RHP] < Checkhp:
		    ref.ref[ha.LHP] = ref.ref[ha.LHP] + HPrate
		    ref.ref[ha.RHP] = ref.ref[ha.RHP] + HPrate
		if p<l:
		    ref.ref[ha.LKN] = ref.ref[ha.LKN] - HPrate
		    p = .34003*np.cos(ref.ref[ha.LHP])+.34038*np.cos(ref.ref[ha.LHP]+ref.ref[ha.LKN])+.11497
		if Checkr >-ref.ref[ha.RAR] :
		    ref.ref[ha.RAR] = ref.ref[ha.RAR] - Rrate
		    ref.ref[ha.LAR] = ref.ref[ha.LAR] - Rrate
		    ref.ref[ha.RHR] = ref.ref[ha.RHR] + Rrate
		    ref.ref[ha.LHR] = ref.ref[ha.LHR] + Rrate

		time.sleep(.0001)
		r.put(ref)
		time.sleep(.5)

	    while ref.ref[ha.LHP] > -.75:
		ref.ref[ha.LHP] = ref.ref[ha.LHP] - .01
		ref.ref[ha.LKN] = ref.ref[ha.LKN] + .01
		r.put(ref)
		time.sleep(.1)


	    print "shifted weight"
	    time.sleep(5)	#GET READY FOR NEXT STEP
	    c1 = 0
	    c2 = 0
	    c3 = 0
	    c4 = 0
	    c5 = 0
	    c6 = 0
	    while c4 + c3 +c5+c6+c1+c2!= 6:
		if ref.ref[ha.RKN] < LKNref -.001:
		    ref.ref[ha.RKN] = ref.ref[ha.RKN]+.0005
		elif ref.ref[ha.RKN] > LKNref +.001: 	  
		    ref.ref[ha.RKN] = ref.ref[ha.RKN]-.0005
		else:
		    c4 = 1 		
		if ref.ref[ha.LKN] < RKNref -.001:
		    ref.ref[ha.LKN] = ref.ref[ha.LKN]+.0005
		elif ref.ref[ha.LKN] > RKNref +.001: 	  
		    ref.ref[ha.LKN] = ref.ref[ha.LKN]-.0005
		else:
		    c3 = 1
		if ref.ref[ha.LHP] < RHPref -.001:
		    ref.ref[ha.LHP] = ref.ref[ha.LHP]+.0005
		elif ref.ref[ha.LHP] > RHPref +.001: 	  
		    ref.ref[ha.LHP] = ref.ref[ha.LHP]-.0005
		else:
		    c6 = 1 
		if ref.ref[ha.RHP] < LHPref -.001:
		    ref.ref[ha.RHP] = ref.ref[ha.RHP]+.0005
		elif ref.ref[ha.RHP] > LHPref +.001: 	  
		    ref.ref[ha.RHP] = ref.ref[ha.RHP]-.0005
		else:
		    c5 = 1 
		if ref.ref[ha.RAP] < LAPref -.001:
		    ref.ref[ha.RAP] = ref.ref[ha.RAP]+.00025
		elif ref.ref[ha.RAP] > LAPref +.001: 	  
		    ref.ref[ha.RAP] = ref.ref[ha.RAP]-.00025
		else:
		    c1 = 1 
		if ref.ref[ha.LAP] < RAPref -.001: 
		    ref.ref[ha.LAP] = ref.ref[ha.LAP]+.00025
		elif ref.ref[ha.LAP] > RAPref +.001: 	  
		    ref.ref[ha.LAP] = ref.ref[ha.LAP]-.00025
		else:
		    c2 = 1
		r.put(ref)
		time.sleep(.01)

	    RAPref = ref.ref[ha.RAP]  #GET STEADY LEG REFERENCE POSITIONS
	    RKNref = ref.ref[ha.RKN]
	    RHPref = ref.ref[ha.RHP]
	    LAPref = ref.ref[ha.LAP]  
	    LKNref = ref.ref[ha.LKN] 
	    LHPref = ref.ref[ha.LHP]  
	    RARref = ref.ref[ha.RAR]
	    LARref = ref.ref[ha.LAR]
	    RHRref = ref.ref[ha.RHR]
	    LHRref = ref.ref[ha.LHR]
	    print 'ready'
	    count = 2
	    finish = finish-1

############################################################################
	    
	while count == 2:  #LEFT STEP
	    l = .34003*np.cos(ref.ref[ha.LHP])+.34038*np.cos(ref.ref[ha.LHP]+ref.ref[ha.LKN])+.11497
	    thetaincrease = np.arcsin(steplength/l)

	    i = 0
	    while i<thetaincrease:	#FIND STEP DISTANCE
		ref.ref[ha.LHP] = ref.ref[ha.LHP]-.005
		ref.ref[ha.LAP] = ref.ref[ha.LAP]+.005
		i = i + .005
		r.put(ref)
		time.sleep(.05)

	    l1 = .34003*np.cos(ref.ref[ha.RHP])+.34038*np.cos(ref.ref[ha.RHP]+ref.ref[ha.RKN])+.11497
	    l2 = .34003*np.cos(ref.ref[ha.LHP])+.34038*np.cos(ref.ref[ha.LHP]+ref.ref[ha.LKN])+.11497
	    

	    while l1 > l2: 	#STEP DOWN
		ref.ref[ha.RAP] = ref.ref[ha.RAP]-.00005
		ref.ref[ha.RKN] = ref.ref[ha.RKN]+.0001
		ref.ref[ha.RHP] = ref.ref[ha.RHP]-.00005
		r.put(ref)
		time.sleep(.00005)
		l1 = .34003*np.cos(ref.ref[ha.RHP])+.34038*np.cos(ref.ref[ha.RHP]+ref.ref[ha.RKN])+.11497
		l2 = .34003*np.cos(ref.ref[ha.LHP])+.34038*np.cos(ref.ref[ha.LHP]+ref.ref[ha.LKN])+.11497
		time.sleep(.0005)

	    print "Stepped Down"

	    p = .34003*np.cos(ref.ref[ha.RHP])+.34038*np.cos(ref.ref[ha.RHP]+ref.ref[ha.RKN])+.11497
	    l = np.sqrt((p)*(p) + (steplength)*(steplength))
	    Checkr = ref.ref[ha.LAR]
	    Checkhp = ref.ref[ha.RHP]
	    Checkap = ref.ref[ha.RAP]
	    Checkkn = .05*(l-p)
	    HPrate = np.abs(.05*(ref.ref[ha.RHP]-ref.ref[ha.LHP]))
	    Rrate = np.abs(.1*Checkr)
	    #SHIFT WEIGHT

	    while Checkr <-ref.ref[ha.LAR] or ref.ref[ha.LHP] < Checkhp or ref.ref[ha.LAP] < Checkap or p<l:
		if ref.ref[ha.LAP] > Checkap:
		    ref.ref[ha.RAP] = -(ref.ref[ha.RHP]+ref.ref[ha.RKN])
		    ref.ref[ha.LAP] = ref.ref[ha.LAP] - HPrate
		if ref.ref[ha.LHP] < Checkhp:
		    ref.ref[ha.RHP] = ref.ref[ha.RHP] + HPrate
		    ref.ref[ha.LHP] = ref.ref[ha.LHP] + HPrate
		if p<l:
		    ref.ref[ha.RKN] = ref.ref[ha.RKN] - HPrate
		    p = .34003*np.cos(ref.ref[ha.RHP])+.34038*np.cos(ref.ref[ha.RHP]+ref.ref[ha.RKN])+.11497
		if Checkr <-ref.ref[ha.LAR] :
		    ref.ref[ha.LAR] = ref.ref[ha.LAR] + Rrate
		    ref.ref[ha.RAR] = ref.ref[ha.RAR] + Rrate
		    ref.ref[ha.LHR] = ref.ref[ha.LHR] - Rrate
		    ref.ref[ha.RHR] = ref.ref[ha.RHR] - Rrate

		time.sleep(.0001)
		r.put(ref)
		time.sleep(.5)

	    while ref.ref[ha.RHP] > -.75:
		ref.ref[ha.RHP] = ref.ref[ha.RHP] - .01
		ref.ref[ha.RAP] = ref.ref[ha.RAP] - .01
		ref.ref[ha.RKN] = ref.ref[ha.RKN] + .01
		i = i + 1
		r.put(ref)
		time.sleep(.1)

	    print "shifted weight"
	    time.sleep(5)	#GET READY FOR NEXT STEP
	    c1 = 0
	    c2 = 0
	    c3 = 0
	    c4 = 0
	    c5 = 0
	    c6 = 0
	    while c4 + c3 +c5+c6+c1+c2!= 6:
		if ref.ref[ha.LKN] < RKNref -.001:
		    ref.ref[ha.LKN] = ref.ref[ha.LKN]+.0005
		elif ref.ref[ha.LKN] > RKNref +.001: 	  
		    ref.ref[ha.LKN] = ref.ref[ha.LKN]-.0005
		else:
		    c4 = 1 		
		if ref.ref[ha.RKN] < LKNref -.001:
		    ref.ref[ha.RKN] = ref.ref[ha.RKN]+.0005
		elif ref.ref[ha.RKN] > LKNref +.001: 	  
		    ref.ref[ha.RKN] = ref.ref[ha.RKN]-.0005
		else:
		    c3 = 1
		if ref.ref[ha.RHP] < LHPref -.001:
		    ref.ref[ha.RHP] = ref.ref[ha.RHP]+.0005
		elif ref.ref[ha.RHP] > LHPref +.001: 	  
		    ref.ref[ha.RHP] = ref.ref[ha.RHP]-.0005
		else:
		    c6 = 1 
		if ref.ref[ha.LHP] < RHPref -.001:
		    ref.ref[ha.LHP] = ref.ref[ha.LHP]+.0005
		elif ref.ref[ha.LHP] > RHPref +.001: 	  
		    ref.ref[ha.LHP] = ref.ref[ha.LHP]-.0005
		else:
		    c5 = 1 
		if ref.ref[ha.LAP] < RAPref -.001:
		    ref.ref[ha.LAP] = ref.ref[ha.LAP]+.00025
		elif ref.ref[ha.LAP] > RAPref +.001: 	  
		    ref.ref[ha.LAP] = ref.ref[ha.LAP]-.00025
		else:
		    c1 = 1 
		if ref.ref[ha.RAP] < LAPref -.001: 
		    ref.ref[ha.RAP] = ref.ref[ha.RAP]+.00025
		elif ref.ref[ha.RAP] > LAPref +.001: 	  
		    ref.ref[ha.RAP] = ref.ref[ha.RAP]-.00025
		else:
		    c2 = 1
		r.put(ref)
		time.sleep(.01)

	    RAPref = ref.ref[ha.RAP]  #GET STEADY LEG REFERENCE POSITIONS
	    RKNref = ref.ref[ha.RKN]
	    RHPref = ref.ref[ha.RHP]
	    LAPref = ref.ref[ha.LAP]  
	    LKNref = ref.ref[ha.LKN] 
	    LHPref = ref.ref[ha.LHP]  
	    RARref = ref.ref[ha.RAR]
	    LARref = ref.ref[ha.LAR]
	    RHRref = ref.ref[ha.RHR]
	    LHRref = ref.ref[ha.LHR]
	    print 'ready'
	    count = 1
	    finish = finish-1
    
