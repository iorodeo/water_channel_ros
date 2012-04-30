"""
create_run_file.py 

Creates an example run specification file for captive trajectory mode. Note, 
it will probably be useful to create of python and matlab classes for generating 
run specification files. 
"""

import h5py
import numpy
import pylab

filename = 'test_position_run_file.hdf5'

f = h5py.File(filename,'w')
num_const = 1
num_ramp = 5
num_cosine = 5 
num_array = 10

info_grp = f.create_group('/info')
info_grp.attrs['mode'] = 'position trajectory'
info_grp.attrs['notes'] = 'blah, blah, blah, ...'

run_cnt = 0

for i in range(0,num_const):

    run_str = '/run_%d'%(run_cnt,)
    run_grp = f.create_group(run_str)

    run_grp.attrs['type'] = 'constant velocity'
    prm_grp = run_grp.create_group('%s/params'%(run_str,))

    T_ds = prm_grp.create_dataset('T',(1,),'f',maxshape=(1,))
    T_ds[:] = 10.0
    T_ds.attrs['unit'] = 's'

    value_ds = prm_grp.create_dataset('velocity',(1,),'f',maxshape=(1,))
    value_ds[:] = 0.1 
    value_ds.attrs['unit'] = 'm/s'

    run_cnt += 1

for i in range(0,num_ramp): 

    run_str = '/run_%d'%(run_cnt,)
    run_grp = f.create_group(run_str)

    run_grp.attrs['type'] = 'ramp profile'
    prm_grp = run_grp.create_group('%s/params'%(run_str,))

    dist_ds = prm_grp.create_dataset('distance',(1,),'f',maxshape=(1,))
    dist_ds[:] = 1.0 + (2.0-1.0)*(i/float(num_ramp-1))
    dist_ds.attrs['unit'] = 'm'

    velo_ds = prm_grp.create_dataset('velocity',(1,),'f',maxshape=(1,))
    velo_ds[:] = 0.1 + (0.3-0.1)*(i/float(num_ramp-1))
    velo_ds.attrs['unit'] = 'm/s'

    accel_ds = prm_grp.create_dataset('acceleration',(1,),'f',maxshape=(1,))
    accel_ds[:] = 0.05 
    accel_ds.attrs['unit'] = 'm/s'

    run_cnt += 1


for i in range(num_cosine):

    run_str = '/run_%d'%(run_cnt,)
    run_grp = f.create_group(run_str)

    run_grp.attrs['type'] = 'cosine'
    prm_grp = run_grp.create_group('%s/params'%(run_str,))

    amp_ds = prm_grp.create_dataset('amplitude',(1,),'f',maxshape=(1,))
    amp_ds[:] = 1.0 + (2.0 - 1.0)*(i/float(num_cosine-1))
    amp_ds.attrs['unit'] = 'm'

    period_ds = prm_grp.create_dataset('period',(1,),'f',maxshape=(1,))
    period_ds[:] = 20.0 
    period_ds.attrs['unit'] = 's'

    cycles_ds = prm_grp.create_dataset('cycles',(1,),'f',maxshape=(1,))
    cycles_ds[:] = 3.0
    cycles_ds.attrs['unit'] = 'qty'

    run_cnt += 1

for i in range(num_array):

    run_str = '/run_%d'%(run_cnt,)
    run_grp = f.create_group(run_str)

    run_grp.attrs['type'] = 'array'

    cycles = 4
    period = 30.0
    dt = 1/50.0
    N = int(numpy.floor(cycles*period/dt))
    t = numpy.arange(0,N)*dt
    x0 = 1.0*numpy.cos(2.0*numpy.pi*t/period)
    x1 = 0.9*numpy.cos(3.0*numpy.pi*t/period)

    array_values = x0 + x1 
    #if i == 0:
    #    pylab.plot(t,array_values)
    #    pylab.show()
    array_values_ds = run_grp.create_dataset(
            'params',
            array_values.shape,
            dtype=array_values.dtype,
            maxshape=array_values.shape
            ) 

    array_values_ds[:] = array_values 
    array_values_ds.attrs['unit'] = 'm'
    run_cnt += 1
