"""
hdf5_run_reader.py

Support class for reading hdf5 run files. 

"""
import os 
import os.path
import h5py

class HDF5_Run_Reader(object):

    def __init__(self,filename):
        self.filename = filename
        self.h5file = h5py.File(self.filename,'r')
        self.pos= 0
        self.run_list = [x for x in list(self.h5file) if x[:3] == 'run']
        self.number_of_runs = len(self.run_list)

    def close(self):
        self.h5file.close()

    def __iter__(self):
        return self

    def get_info(self):
        return self.h5file['/info']

    def get_mode(self):
        return self.h5file['/info'].attrs['mode']

    def next(self):
        if self.pos < self.number_of_runs:
            run_str = '/run_%d'%(self.pos,)
            self.pos += 1
            return self.h5file[run_str]
        else:
            raise StopIteration

    def get_run(self,n):
        if n >=0 and n < self.number_of_runs:
            run_str = '/run_%d'%(n,)
            self.pos = n+1
            return self.h5file[run_str]
        else:
            return None

    def rewind(self):
        self.pos = 0

    def move_to(self,pos):
        if pos < 0 or pos > self.number_of_runs:
            raise ValueError, 'run position out of range'
        else:
            self.pos = pos

    def print_struct(self):
        print 
        print 'File: %s'%(self.filename,)
        print '-'*60
        print 'printing structure ...'
        print
        print_node_struct(self.h5file['/'])

def print_node_struct(node):
    """
    Recursively prints the structure of a hdf5 file node.
    """
    child_list = list(node)
    for child in child_list:
        print node[child].name 
        print_node_attrs(node[child])
        if not isinstance(node[child],h5py.highlevel.Dataset):
            print_node_struct(node[child])
        else:
            shape = node[child].shape
            if shape[0] == 1:
                print ' value:', float(node[child][0])
            else:
                print ' shape:', node[child].shape

def print_node_attrs(node):
    """
    Prints the attributes of hdf5 file node.
    """
    for k,v in node.attrs.iteritems():
        print ' %s: %s'%(k,v)

# ---------------------------------------------------------------------------------------
if __name__ == '__main__':

    import sys

    filename = sys.argv[1]
    run_reader = HDF5_Run_Reader(filename)

    if 0:
        for run in run_reader:
            print_node_struct(run)
            print 
    if 1:
        for i in range(0,run_reader.number_of_runs):
            run = run_reader.get_run(i)
            print_node_struct(run)
            print
    
            
        


