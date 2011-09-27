"""
create_run_file.py 

Creates an example run specification file for captive trajectory mode. Note, 
it will probably be useful to create of python and matlab classes for generating 
run specification files. 
"""

import h5py
import numpy.random

filename = 'test_run_file.hdf5'
num_runs = 10

f = h5py.File(filename,'w')

info_grp = f.create_group('/info')
info_grp.attrs['mode'] = 'captive_trajectory'
info_grp.attrs['notes'] = 'blah, blah, blah, ...'

for i in range(0,num_runs):

    run_str = '/run_%d'%(i,)
    run_grp = f.create_group(run_str)
    
    mass_ds = run_grp.create_dataset('mass',(1,),'f',maxshape=(1,))
    mass_ds[:] = 2.0*i
    mass_ds.attrs['unit'] = 'kg'

    damping_ds = run_grp.create_dataset('damping',(1,),'f',maxshape=(1,))
    damping_ds[:] = 0.0
    damping_ds.attrs['unit'] = 'kg/s'

    if i < num_runs/3:
        run_grp.attrs['type'] = 'constant'

        T_ds = run_grp.create_dataset('T',(1,),'f',maxshape=(1,))
        T_ds[:] = 10.0
        T_ds.attrs['unit'] = 's'

        value_ds = run_grp.create_dataset('value',(1,),'f',maxshape=(1,))
        value_ds[:] = 500.0
        value_ds.attrs['unit'] = 'us'

    elif i < 2*num_runs/3:
        run_grp.attrs['type'] = 'trapeziodal'

        prm_grp = run_grp.create_group('%s/params'%(run_str,))
        prm_grp.attrs['T'] = 10.0
        prm_grp.attrs['value'] = 300.0
        prm_grp.attrs['rate'] = 100.0
    else:
        run_grp.attrs['type'] = 'array'
        array_values = numpy.random.rand(500)
        array_values_ds = run_grp.create_dataset(
                'values',
                array_values.shape,
                dtype=array_values.dtype,
                maxshape=array_values.shape
                ) 
        array_values_ds[:] = array_values 
