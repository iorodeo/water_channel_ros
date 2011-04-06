"""
Copyright 2010  IO Rodeo Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""
from __future__ import division
import roslib
roslib.load_manifest('water_channel_mechanics')
import rospy
import math
import copy

import cad.csg_objects as csg
import cad.finite_solid_objects as fso
import cad.pattern_objects as po
import cad_library.origin as origin

import laser_sensor_short_range

LASER_SENSOR_SHORT_RANGE_MOUNT_PLATE_PARAMETERS = {
    'color': [0.98,0.98,0.98,1.0],
    'x': 3.0,
    'y': 0.236,
    'z': 4.0,
    'lssr_tx': -0.5,
    'lssr_tz': -0.75,
    'mount_hole_tx': 0.5,
    'hole_r_big': 0.125,
    'hole_y_big': 2.5,
    'hole_r_lssr': 0.0625,
    'hole_l': 1.0,
    'show_origin': False,
    }

def get_parameters():
    return copy.deepcopy(LASER_SENSOR_SHORT_RANGE_MOUNT_PLATE_PARAMETERS)

class LaserSensorShortRangeMountPlate(csg.Union):
    def __init__(self):
        super(LaserSensorShortRangeMountPlate, self).__init__()
        self.parameters = LASER_SENSOR_SHORT_RANGE_MOUNT_PLATE_PARAMETERS
        self.lssr_parameters = laser_sensor_short_range.get_parameters()
        self.__make_laser_sensor_short_range_mount_plate()
        self.__make_origin()

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_laser_sensor_short_range_mount_plate(self):
        length = self.parameters['x']
        width = self.parameters['y']
        height = self.parameters['z']
        laser_sensor_short_range_mount_plate = fso.Box(x=length,y=width,z=height)

        hole_l = self.parameters['hole_l']

        # Laser_sensor_short_range mount holes
        hole_r_laser_sensor_short_range = self.parameters['hole_r_lssr']
        base_hole = fso.Cylinder(r=hole_r_laser_sensor_short_range,l=hole_l)
        base_hole.rotate(angle=math.pi/2,axis=[1,0,0])

        hole_x = self.lssr_parameters['hole_ty']
        hole_z = self.lssr_parameters['hole_tx']
        holes = po.LinearArray(base_hole,[-hole_x,hole_x],0,[-hole_z,hole_z])
        holes.translate([self.parameters['lssr_tx'],0,self.parameters['lssr_tz']])
        laser_sensor_short_range_mount_plate -= holes

        # T-slotted bracket mount holes
        hole_r_big = self.parameters['hole_r_big']
        base_hole = fso.Cylinder(r=hole_r_big,l=hole_l)
        base_hole.rotate(angle=math.pi/2,axis=[1,0,0])

        hole_ax = [-0.5,0.5]
        hole_az = [-1.5,0,1.5]
        holes = po.LinearArray(base_hole,hole_ax,0,hole_az)
        holes.translate([self.parameters['mount_hole_tx'],0,0])
        laser_sensor_short_range_mount_plate -= holes

        laser_sensor_short_range_mount_plate.set_color(self.parameters['color'],recursive=True)
        self.add_obj(laser_sensor_short_range_mount_plate)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    laser_sensor_short_range_mount_plate = LaserSensorShortRangeMountPlate()
    laser_sensor_short_range_mount_plate.export()





