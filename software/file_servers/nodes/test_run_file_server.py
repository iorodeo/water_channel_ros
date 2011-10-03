#!/usr/bin/env python
"""
"""
import roslib 
roslib.load_manifest('file_servers')
import rospy
import sys
import numpy

# Services
from msg_and_srv.srv import SetRunFile 
from msg_and_srv.srv import GetRunFile
from msg_and_srv.srv import ClearRunFile
from msg_and_srv.srv import GetRunData

def set_run_file(filename):
    rospy.wait_for_service('set_run_file')
    try:
        set_run_file_cmd = rospy.ServiceProxy('set_run_file', SetRunFile)
        response = set_run_file_cmd(filename)
        print ' status:         ', response.status
        print ' message:        ', response.message
        print ' mode:           ', response.mode
        print ' number of runs: ', response.number_of_runs
        print
    except rospy.ServiceException, e:
        print 'set_run_file service call failed: %s'%(e,)
        response = None
    return response

def get_run_file():
    rospy.wait_for_service('get_run_file')
    try:
        get_run_file_cmd = rospy.ServiceProxy('get_run_file',GetRunFile)
        response = get_run_file_cmd()
        print ' status:         ', response.status
        print ' message:        ', response.message
        print ' filename:       ', response.filename
        print ' mode:           ', response.mode
        print ' number of runs: ', response.number_of_runs
        print
    except rospy.ServiceException, e:
        print 'get_run_file service call failed: %s'%(e,)
        response = None
    return response

def clear_run_file():
    rospy.wait_for_service('clear_run_file')
    try:
        clear_run_file_cmd = rospy.ServiceProxy('clear_run_file', ClearRunFile)
        response = clear_run_file_cmd()
        print ' status:         ', response.status
        print
    except rospy.ServiceException, e:
        print 'clear_run_file service call failed: %s'%(e,)
    return response

def get_run_data(run_number):
    rospy.wait_for_service('get_run_data')
    try:
        get_run_data_cmd = rospy.ServiceProxy('get_run_data', GetRunData)
        response = get_run_data_cmd(run_number)
        print ' status:         ', response.status
        print ' message:        ', response.message
        print ' filename:       ', response.filename
        print ' values:         ', 'shape =', numpy.array(response.values).shape
        print ' mass:           ', response.mass
        print ' damping:        ', response.damping
        print
        values = numpy.array(response.values)
        N = values.shape[0]
        dt = 1.0/50.0
        t = dt*pylab.arange(N)
        dvalues = (values[1:] - values[:-1])/dt
        #pylab.plot(t[1:],dvalues)
        pylab.plot(t,values)
        pylab.show()
    except rospy.ServiceException, e:
        print 'get_run_data service call failed: %s'(e,) 
    return response

# -----------------------------------------------------------------------------
import pylab

filename = sys.argv[1]

print 'Set run file to existing file: %s'%(filename,)
resp = set_run_file(filename)

print 'Set run file to non-existant file'
resp = set_run_file('slkfjslsfj')

print 'Set run file to existing file: %s'%(filename,)
resp = set_run_file(filename)

print 'Get run file info'
resp = get_run_file()

print 'Get run data from file'
resp = get_run_data(0)

print 'Clear run file'
resp = clear_run_file()

print 'Get run file info'
resp = get_run_file()
