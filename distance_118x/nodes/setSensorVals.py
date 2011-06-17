from distance_sensor_118x import DistanceSensor

sensor = DistanceSensor('/dev/USB_Distance')
sensor.open()
sensor.setAveraging(5)
sensor.printSettings()
sensor.close()
