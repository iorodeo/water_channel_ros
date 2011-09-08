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
        self.redata_size_incr = 1000
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
        for name, data_size in self.data_size.iteritems():
            elem_shape = self.f['data'][name].shape[1:]
            new_shape = (data_size,) + elem_shape
            self.f['data'][name].resize(new_shape)

    def addInfoAttribute(self,name,value):
        """
        Add attribute to log information group
        """
        self.f['info'].attrs[name] = value

    def addDataSet(self,name,elem_shape,dtype='f'):
        """
        Adds dataset to hdf5 log file.
        """
        inishape = (self.redata_size_incr,) + elem_shape
        maxshape = (None,) + elem_shape
        self.f['data'].create_dataset(name, inishape, dtype, maxshape=maxshape)
        self.data_size[name] = 0

    def addDataAttribute(self,name,attribute,value):
        """
        Adds attribute to hdf5 dataset.
        """
        self.f['data'][name].attrs[attribute] = value
    
    def addDataValue(self,name,value):
        """
        Adds value to log dataset.
        """
        # Resize dataset if necessary
        n = self.f['data'][name].shape[0]
        data_size = self.data_size[name]
        if data_size >= n:
            elem_shape = self.f['data'][name].shape[1:]
            new_shape = (n+self.redata_size_incr,) + elem_shape
            self.f['data'][name].redata_size(new_shape)
        # Add item to dataset
        self.f['data'][name][data_size]= value
        self.data_size[name] +=1

# -----------------------------------------------------------------------------
if __name__ == '__main__':


    logger = HDF5_Logger('sample.hdf5') 
    logger.open()
    logger.addDataSet('analog_input', (1,))
    logger.addDataSet('samples', (2,))
    logger.addDataSet('tests', (2,2))
    logger.addInfoAttribute('notes', 'This experiment blah, blah, etc.')
    logger.addDataAttribute('analog_input', 'unit', 'V')
    logger.addDataAttribute('samples', 'unit', 'N')
    logger.addDataAttribute('tests', 'unit', 'frog/sec')
    
    
    for i in range(0,500):
        logger.addDataValue('analog_input', 1.0*i)
        logger.addDataValue('samples', (2.0*i, 3.0*i))
        logger.addDataValue('tests', [[i,i+1],[2*i, 3*i]])
    
    logger.close()






