import pylab
from distance_sensor_118x import DistanceSensor

sensor = DistanceSensor('/dev/USB_Distance')
sensor.open()

sensor.startDistTracking('50hz')
N = 500
data = pylab.zeros((N,)) 
for i in range(0,N):
    value = sensor.readSample(convert='float')
    print 'convert', value
    if not value is None:
        data[i] = value
    else:
        data[i] = 0.0


sensor.laserOff()
sensor.close()

pylab.figure(1)
pylab.plot(data)
pylab.show()

