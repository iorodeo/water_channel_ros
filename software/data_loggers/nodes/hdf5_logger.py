import h5py
import time
import datetime

class HDF5_Logger(object):
    """
    Simple HDF5 data logger class
    """

    def __init__(self,filename):
        self.filename = filename
        self.data_size = {}
        self.resize_incr = 10000
        self.date_format = '%m-%d-%Y %H:%M:%S'

    def open(self,add_datetime=True):
        """
        Opens hdf5 log file. Optionally appends date to it.
        """
        self.f = h5py.File(self.filename,'w')
        self.f.create_group('info')
        self.f.create_group('data')
        if add_datetime==True:
            date_string = datetime.datetime.now().strftime(self.date_format)
            self.f['info'].attrs['date_string'] = date_string 

    def close(self):
        """
        Closes hdf4 log file.
        """
        for pathname, data_size in self.data_size.iteritems():
            data_set = self._getDataSet(pathname)
            elem_shape = data_set.shape[1:]
            new_shape = (data_size,) + elem_shape
            data_set.resize(new_shape)

    def addInfoAttribute(self,name,value):
        """
        Add attribute to log information group
        """
        self.f['info'].attrs[name] = value

    def _getSubgroupAndName(self,pathname):
        subgroup = self.f['data']
        if type(pathname) == str:
            name = pathname
        else:
            for item in pathname[:-1]:
                subgroup = subgroup[item]
            name = pathname[-1]
        return subgroup, name

    def _getDataSet(self,pathname):
        if type(pathname) == str:
            data_set = self.f['data'][pathname]
        else:
            data_set = self.f['data']
            for item in pathname:
                data_set = data_set[item]
        return data_set

    def _getDataSize(self, pathname):
        if type(pathname) == str:
            data_size = self.data_size[pathname]
        else:
            data_size = self.data_size[tuple(pathname)]
        return data_size

    def _setDataSize(self,pathname, value):
        if type(pathname) == str:
            self.data_size[pathname] = value
        else:
            self.data_size[tuple(pathname)] = value

    def addDataGroup(self,pathname):
        """
        Add a data group to the log
        """
        subgroup, name = self._getSubgroupAndName(pathname)
        subgroup.create_group(name)

    def addDataSet(self,pathname,elem_shape,dtype='f'):
        """
        Adds dataset to hdf5 log file.
        """
        subgroup, name = self._getSubgroupAndName(pathname)
        inishape = (self.resize_incr,) + elem_shape
        maxshape = (None,) + elem_shape
        subgroup.create_dataset(name, inishape, dtype, maxshape=maxshape)
        self._setDataSize(pathname,0)

    def addDataAttribute(self,pathname,attribute,value):
        """
        Adds attribute to hdf5 dataset.
        """
        dataSet = self._getDataSet(pathname)
        dataSet.attrs[attribute] = value
    
    def addDataValue(self,pathname,value):
        """
        Adds value to log dataset.  """
        data_set = self._getDataSet(pathname)
        # Resize dataset if necessary
        n = data_set.shape[0]
        data_size = self._getDataSize(pathname)
        if data_size >= n:
            print 'resizing'
            elem_shape = data_set.shape[1:]
            new_shape = (n+self.resize_incr,) + elem_shape
            data_set.resize(new_shape)
        # Add item to dataset
        data_set[data_size]= value
        self._setDataSize(pathname,data_size+1)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import time


    logger = HDF5_Logger('sample.hdf5') 
    logger.open()
    logger.addDataSet('analog_input', (1,))
    logger.addDataSet('samples', (2,))
    logger.addDataSet('tests', (2,2))

    logger.addDataGroup('subgroup1')
    logger.addDataSet(['subgroup1', 'data1'], (1,))

    logger.addInfoAttribute('notes', 'This experiment blah, blah, etc.')
    logger.addDataAttribute('analog_input', 'unit', 'V')
    logger.addDataAttribute('samples', 'unit', 'N')
    logger.addDataAttribute('tests', 'unit', 'frog/sec')
    
    
    for i in range(0,5000):
        t0 = time.time()
        logger.addDataValue('analog_input', 1.0*i)
        logger.addDataValue('samples', (2.0*i, 3.0*i))
        logger.addDataValue('tests', [[i,i+1],[2*i, 3*i]])
        logger.addDataValue(['subgroup1', 'data1'], 3.0*i)
        t1 = time.time()
        dt = t1 -t0
        print dt, 1.0/dt
    
    logger.close()






