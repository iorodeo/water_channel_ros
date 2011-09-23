"""
hdf5_read_test.py

This script tests reading the hdf5 log files created by the various logging
nodes.  
""" 
import sys 
import h5py

filename = sys.argv[1]

f = h5py.File(filename,'r')

def print_file_struct(group,show_attrs=False):
    group_list = list(group)
    try:
        for name, obj in group.iteritems():
            print obj.name, 
            if isinstance(obj,h5py.highlevel.Dataset):
                print '(D)'
            else:
                print '(G)'
            if show_attrs:
                for k,v in obj.attrs.iteritems():
                    print ' A: key = %s, value = %s'%(k, v)
            if not isinstance(obj,h5py.highlevel.Dataset):
                print_file_struct(obj,show_attrs=show_attrs)
    except:
        pass

print
print 'log file strucuture'
print_file_struct(f,show_attrs=False)

if 1:
    import pylab
    t = f['/trial_0/data/time']
    print t.shape
    values = f['trial_0/data/actuator']
    t = pylab.array(t)
    t = t.reshape((t.shape[0],))
    dt = t[1:] - t[:-1]

    pylab.figure(1)
    pylab.plot(t,values)
    #pylab.figure(2)
    #pylab.plot(dt/dt.mean())
    pylab.show()

if 0:
    print 'mass:', f['/data/mass'][0]
    print 'damping:', f['/data/damping'][0]


f.close()
