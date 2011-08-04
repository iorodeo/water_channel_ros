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
import cad.export.bom as bom

LASER_SENSOR_LONG_RANGE_PARAMETERS = {
    'color': [0.2,0.2,0.2,1.0],
    'x': 5.670,
    'y': 3.898,
    'z': 2.008,
    'lens_r': 0.984,
    'lens_l': 1.969,
    'lens_ty': -0.315,
    'hole_r': 0.098,
    'hole_l': 0.25,
    'hole_x': 0.709,
    'hole_y': 1.661,
    'show_origin': False,
    'laser_beam_r': 0.1,
    'laser_beam_l': 35,
    'laser_beam_color': [1,0,0,1],
    'show_laser_beam': True,
    }

def get_parameters():
    return copy.deepcopy(LASER_SENSOR_LONG_RANGE_PARAMETERS)

class LaserSensorLongRange(csg.Difference):
    def __init__(self):
        super(LaserSensorLongRange, self).__init__()
        self.parameters = LASER_SENSOR_LONG_RANGE_PARAMETERS
        self.__make_laser_sensor_long_range()
        self.__make_holes()
        self.__set_bom()
        self.__make_origin()

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_laser_sensor_long_range(self):
        x = self.parameters['x']
        y = self.parameters['y']
        z = self.parameters['z']
        lslr = fso.Box(x=x,y=y,z=z)

        lens_r = self.parameters['lens_r']
        lens_l = self.parameters['lens_l']
        lens = fso.Cylinder(r=lens_r,l=lens_l)
        lens.rotate(angle=-math.pi/2,axis=[0,1,0])
        lens_tx = -lens_l/2 - x/2
        lens_ty = self.parameters['lens_ty']
        lens.translate([lens_tx,lens_ty,0])
        lslr |= lens
        lslr.set_color(self.parameters['color'],recursive=True)

        if self.parameters['show_laser_beam']:
            laser_beam = fso.Cylinder(r=self.parameters['laser_beam_r'],l=self.parameters['laser_beam_l'])
            laser_beam.rotate(angle=math.pi/2,axis=[0,1,0])
            laser_beam.set_color(self.parameters['laser_beam_color'])
            laser_beam_tx = -self.parameters['laser_beam_l']/2
            laser_beam_ty = lens_ty
            laser_beam_tz = 0
            laser_beam.translate([laser_beam_tx,laser_beam_ty,laser_beam_tz])
            lslr |= laser_beam

        self.add_obj(lslr)

    def __make_holes(self):
        hole_r = self.parameters['hole_r']
        hole_l = self.parameters['hole_l']
        hole = fso.Cylinder(r=hole_r,l=hole_l*2)
        hole_x = self.parameters['hole_x']
        hole_y = self.parameters['hole_y']
        hole_z = 0
        holes = po.LinearArray(hole,x=[-hole_x,hole_x],y=[-hole_y,hole_y],z=hole_z)
        holes.translate([0,0,-self.parameters['z']/2])
        holes.set_color(self.parameters['color'],recursive=True)
        self.add_obj(holes)

    def __set_bom(self):
        scale = self.get_scale()
        BOM = bom.BOMObject()
        BOM.set_parameter('name','laser_sensor_long_range')
        BOM.set_parameter('description','Micro-Epsilon optoNCDT ILR long range distance sensor')
        BOM.set_parameter('dimensions','x: {x:0.3f}, y: {y:0.3f}, z: {z:0.3f}'.format(x=self.parameters['x']*scale[0],y=self.parameters['y']*scale[1],z=self.parameters['z']*scale[2]))
        BOM.set_parameter('vendor','Micro-Epsilon')
        BOM.set_parameter('part number','ILR1182-30(01)')
        self.set_object_parameter('bom',BOM)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    laser_sensor_long_range = LaserSensorLongRange()
    laser_sensor_long_range.export()





