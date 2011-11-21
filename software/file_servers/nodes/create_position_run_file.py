"""
create_run_file.py 

Creates an example run specification file for captive trajectory mode. Note, 
it will probably be useful to create of python and matlab classes for generating 
run specification files. 
"""

import h5py
import numpy.random

filename = 'test_run_file.hdf5'
num_runs = 15

f = h5py.File(filename,'w')

info_grp = f.create_group('/info')
info_grp.attrs['mode'] = 'position trajectory'
info_grp.attrs['notes'] = 'blah, blah, blah, ...'

for i in range(0,num_runs):

    run_str = '/run_%d'%(i,)
    run_grp = f.create_group(run_str)

    if i < num_runs/4:
        run_grp.attrs['type'] = 'constant velocity'
        prm_grp = run_grp.create_group('%s/params'%(run_str,))

        T_ds = prm_grp.create_dataset('T',(1,),'f',maxshape=(1,))
        T_ds[:] = 10.0
        T_ds.attrs['unit'] = 's'

        value_ds = prm_grp.create_dataset('velocity',(1,),'f',maxshape=(1,))
        value_ds[:] = 0.1 
        value_ds.attrs['unit'] = 'm/s'

    elif i < 2*num_runs/4:
        run_grp.attrs['type'] = 'ramp profile'
        prm_grp = run_grp.create_group('%s/params'%(run_str,))

        dist_ds = prm_grp.create_dataset('distance',(1,),'f',maxshape=(1,))
        dist_ds[:] = 2.0
        dist_ds.attrs['unit'] = 'm'

        velo_ds = prm_grp.create_dataset('velocity',(1,),'f',maxshape=(1,))
        velo_ds[:] = 0.1
        velo_ds.attrs['unit'] = 'm/s'

        accel_ds = prm_grp.create_dataset('acceleration',(1,),'f',maxshape=(1,))
        accel_ds[:] = 0.1 
        accel_ds.attrs['unit'] = 'm/s'


    elif i < 3*num_runs/4:
        run_grp.attrs['type'] = 'sinewave'
        prm_grp = run_grp.create_group('%s/params'%(run_str,))

        amp_ds = prm_grp.create_dataset('amplitude',(1,),'f',maxshape=(1,))
        amp_ds[:] = 2.0
        amp_ds.attrs['unit'] = 'm'

        period_ds = prm_grp.create_dataset('period',(1,),'f',maxshape=(1,))
        period_ds[:] = 20.0 
        period_ds.attrs['unit'] = 's'

        cycles_ds = prm_grp.create_dataset('cycles',(1,),'f',maxshape=(1,))
        cycles_ds[:] = 5.0
        cycles_ds.attrs['unit'] = 'qty'

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
        array_values_ds.attrs['unit'] = 'm'
