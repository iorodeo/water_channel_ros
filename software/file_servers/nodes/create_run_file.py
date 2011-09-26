import h5py
import numpy.random


filename = 'test_run_file.hdf5'
num_runs = 10

f = h5py.File(filename,'w')

info_grp = f.create_group('/info')
info_grp.attrs['mode'] = 'captive'
info_grp.attrs['notes'] = 'blah, blah, blah, ...'

for i in range(0,num_runs):

    run_str = '/run_%d'%(i,)
    run_grp = f.create_group(run_str)
    run_grp.attrs['mass'] = 2.0*i
    run_grp.attrs['damping'] = 0.0

    if i < num_runs/2:
        run_grp.attrs['type'] = 'ramp'
        prm_grp = run_grp.create_group('%s/params'%(run_str,))
        prm_grp['start'] = 0
        prm_grp['end'] = 400
        prm_grp['velo'] = 500.0
        prm_grp['accel'] = 100.0
        
    else:
        run_grp.attrs['type'] = 'array'
        values = numpy.random.rand(500)
        dataset = run_grp.create_dataset('values',array.shape,dtype=array.dtype,maxshape=array.shape) 
        dataset[:] = array



