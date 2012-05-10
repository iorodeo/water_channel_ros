from distance_sensor_118x import DistanceSensor
sensor = DistanceSensor('/dev/USB_Distance')
sensor.open()
sensor.printSettings()
sensor.close()
