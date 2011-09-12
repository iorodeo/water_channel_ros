import h5py

f = h5py.File('sample.hdf5','r')

def print_data(group,indent=''):
    group_list = list(group)
    try:
        for name, obj in group.iteritems():
            if not isinstance(obj,h5py.highlevel.Dataset):
                print
                print '%s * G %s'%(indent,name)
                for k,v in obj.attrs.iteritems():
                    print '%s A: key = %s, value = %s'%(indent + 2*' ', k, v)
                print_data(obj,indent=indent + ' ')
            else:
                print '%s * D %s, shape = %s'%(indent, name, obj.shape)
                for k,v in obj.attrs.iteritems():
                    print '%s A: key = %s, value = %s'%(indent + 2*' ', k, v)
    except:
        pass

print
print 'log file strucuture'
print_data(f)

f.close()
