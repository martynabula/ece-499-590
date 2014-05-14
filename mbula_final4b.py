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

count = 0

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=False)

i = 0
while i < 15:
    ref.ref[ha.RSR] = ref.ref[ha.RSR]-.01
    ref.ref[ha.LSR] = ref.ref[ha.LSR]+.01
    i = i+.5
    r.put(ref)
    time.sleep(.01)
#while i < 5:
    #ref.ref[ha.RSP] = ref.ref[ha.RSP]-.01
    #ref.ref[ha.LSP] = ref.ref[ha.LSP]+.01
    #i = i+.5
    #r.put(ref)
    #time.sleep(.01)
k = 0
while k < 5:
	while count == 0:
	    i = 0		
	    while i < .18: 	
		ref.ref[ha.RAR] = i
		ref.ref[ha.LAR] = i
		ref.ref[ha.RHR] = -i
		ref.ref[ha.LHR] = -i
		r.put(ref)
		i = i+ .01
		time.sleep(.5)
	    i = 0
	    while i <.9: 
		ref.ref[ha.RAP] = -i
		ref.ref[ha.RKN] = 2*i
		ref.ref[ha.RHP] = -i
		r.put(ref)
		i = i+.05
		time.sleep(.5)
	    count = 1 
	    
	i = 0
	while i < .78: 	
		ref.ref[ha.LAP] = -i
		ref.ref[ha.LKN] = 2*i
		ref.ref[ha.LHP] = -i
		r.put(ref)
		i = i+ .01
		time.sleep(.1)
	i = 0.78
	while i > 0: 	
		ref.ref[ha.LAP] = -i
		ref.ref[ha.LKN] = 2*i
		ref.ref[ha.LHP] = -i
		r.put(ref)
		i = i- .01
		time.sleep(.1)   
	k = k + 1;
while 1:
	i = 0.9	
	while i > 0: 
		ref.ref[ha.RAP] = -i
		ref.ref[ha.RKN] = 2*i
		ref.ref[ha.RHP] = -i
		r.put(ref)
		i = i-.05
		time.sleep(.5)
while 1:
	i = 0.18
	while i < .18: 	
		ref.ref[ha.RAR] = i
		ref.ref[ha.LAR] = i
		ref.ref[ha.RHR] = -i
		ref.ref[ha.LHR] = -i
		r.put(ref)
		i = i- .01
		time.sleep(.5)
	    


