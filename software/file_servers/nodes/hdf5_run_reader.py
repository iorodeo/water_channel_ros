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

    def get_info(self):
        return self.h5file['/info']

    def get_next(self):
        if self.pos < self.number_of_runs:
            run_str = '/run_%d'%(self.pos,)
            self.pos += 1
            return self.h5file[run_str]
        else:
            return None

    def get_run(self,n):
        if n >=0 and n < self.number_of_runs:
            run_str = '/run_%d'%(n,)
            self.pos = n+1
            return self.h5file[run_str]
        else:
            return None

    def rewind(self):
        self.pos = 0


