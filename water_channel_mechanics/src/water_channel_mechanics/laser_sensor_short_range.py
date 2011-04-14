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
import cad.csg_objects as csg
import cad.finite_solid_objects as fso
import cad.export.bom as bom

import math
import copy


LASER_SENSOR_SHORT_RANGE_PARAMETERS = {
    'x': 2.559,
    'y': 1.969,
    'z': 0.787,
    'profile': 'laser_sensor_short_range/laser_sensor_short_range.dxf',
    'color': [0.2,0.2,0.2,1.0],
    'hole_tx': 1.122,
    'hole_ty': 0.787,
    'hole_r': 0.085,
    'hole_l': 4,
    'laser_beam_tx': 0.65,
    'laser_beam_r': 0.1,
    'laser_beam_l': 2,
    'laser_beam_color': [1,0,0,1],
    'show_laser_beam': True,
    }

def get_parameters():
    return copy.deepcopy(LASER_SENSOR_SHORT_RANGE_PARAMETERS)

class LaserSensorShortRange(csg.Difference):
    def __init__(self):
        super(LaserSensorShortRange, self).__init__()
        self.parameters = LASER_SENSOR_SHORT_RANGE_PARAMETERS
        self.__make_laser_sensor_short_range()
        self.__make_holes()
        self.__set_bom()
        self.rotate(angle=-math.pi/2,axis=[1,0,0])
        self.rotate(angle=-math.pi/2,axis=[0,1,0])

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_laser_sensor_short_range(self):
        profile = self.parameters['profile']
        l = self.parameters['z']
        lssr = fso.Extrusion(profile=profile,l=l)
        lssr.set_color(self.parameters['color'],recursive=True)

        if self.parameters['show_laser_beam']:
            laser_beam = fso.Cylinder(r=self.parameters['laser_beam_r'],l=self.parameters['laser_beam_l'])
            laser_beam.rotate(angle=math.pi/2,axis=[1,0,0])
            laser_beam.set_color(self.parameters['laser_beam_color'])
            laser_beam_tx = self.parameters['laser_beam_tx']
            laser_beam_ty = -self.parameters['laser_beam_l']/2
            laser_beam_tz = 0
            laser_beam.translate([laser_beam_tx,laser_beam_ty,laser_beam_tz])
            lssr |= laser_beam

        self.add_obj(lssr)

    def __make_holes(self):
        hole_r = self.parameters['hole_r']
        hole_l = self.parameters['hole_l']
        hole_tx = self.parameters['hole_tx']
        hole_ty = self.parameters['hole_ty']
        base_hole = fso.Cylinder(r=hole_r,l=hole_l)
        hole1 = base_hole.copy()
        hole1.translate([-hole_tx,hole_ty,0])
        hole2 = base_hole.copy()
        hole2.translate([hole_tx,-hole_ty,0])
        hole1.set_color(self.parameters['color'])
        hole2.set_color(self.parameters['color'])
        self.add_obj([hole1,hole2])

    def __set_bom(self):
        scale = self.get_scale()
        BOM = bom.BOMObject()
        BOM.set_parameter('name','laser_sensor_short_range')
        BOM.set_parameter('description','Micro-Epsilon short range distance sensor')
        BOM.set_parameter('dimensions','100mm')
        BOM.set_parameter('vendor','Micro-Epsilon')
        BOM.set_parameter('part number','optoNCDT1302')
        self.set_object_parameter('bom',BOM)


if __name__ == "__main__":
    import cad_library.origin as origin
    o = origin.Origin()
    laser_sensor_short_range = LaserSensorShortRange()
    laser_sensor_short_range |= o
    laser_sensor_short_range.export()





