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

import laser_sensor_short_range

LASER_SENSOR_SHORT_RANGE_MOUNT_PLATE_PARAMETERS = {
    'color': [0.98,0.98,0.98,1.0],
    'x': 3.0,
    'y': 4.0,
    'z': 0.236,
    'lssr_tx': -0.5,
    'lssr_ty': -0.75,
    'mount_hole_tx': 0.5,
    'hole_r_big': 0.125,
    'hole_y_big': 2.5,
    'hole_r_lssr': 0.0625,
    'hole_l': 1.0,
    'show_origin': False,
    }

def get_parameters():
    return copy.deepcopy(LASER_SENSOR_SHORT_RANGE_MOUNT_PLATE_PARAMETERS)

class LaserSensorShortRangeMountPlate(csg.Difference):
    def __init__(self):
        super(LaserSensorShortRangeMountPlate, self).__init__()
        self.parameters = LASER_SENSOR_SHORT_RANGE_MOUNT_PLATE_PARAMETERS
        self.lssr_parameters = laser_sensor_short_range.get_parameters()
        self.__make_laser_sensor_short_range_mount_plate()
        self.__make_holes()
        self.__set_bom()
        self.__make_origin()
        self.set_color(self.parameters['color'],recursive=True)

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_laser_sensor_short_range_mount_plate(self):
        x = self.parameters['x']
        y = self.parameters['y']
        z = self.parameters['z']
        lssrmp = fso.Box(x=x,y=y,z=z)
        self.add_obj(lssrmp)

    def __make_holes(self):
        hole_l = self.parameters['hole_l']

        # Laser_sensor_short_range mount holes
        hole_r_lssr = self.parameters['hole_r_lssr']
        base_hole = fso.Cylinder(r=hole_r_lssr,l=hole_l)

        hole_x = self.lssr_parameters['hole_ty']
        hole_y = self.lssr_parameters['hole_tx']
        holes = po.LinearArray(base_hole,[-hole_x,hole_x],[-hole_y,hole_y],0)
        holes.translate([self.parameters['lssr_tx'],self.parameters['lssr_ty'],0])
        self.add_obj(holes)

        # T-slotted bracket mount holes
        hole_r_big = self.parameters['hole_r_big']
        base_hole = fso.Cylinder(r=hole_r_big,l=hole_l)

        hole_ax = [-0.5,0.5]
        hole_ay = [-1.5,0,1.5]
        holes = po.LinearArray(base_hole,hole_ax,hole_ay,0)
        holes.translate([self.parameters['mount_hole_tx'],0,0])
        self.add_obj(holes)

    def __set_bom(self):
        scale = self.get_scale()
        BOM = bom.BOMObject()
        BOM.set_parameter('name','laser_sensor_short_range_mount_plate')
        BOM.set_parameter('description','Mounts short range laser sensor to t_slotted beams')
        BOM.set_parameter('dimensions','x: {x:0.3f}, y: {y:0.3f}, z: {z:0.3f}'.format(x=self.parameters['x']*scale[0],y=self.parameters['y']*scale[1],z=self.parameters['z']*scale[2]))
        BOM.set_parameter('vendor','Pololu')
        BOM.set_parameter('part number','?')
        BOM.set_parameter('cost',0)
        self.set_object_parameter('bom',BOM)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    laser_sensor_short_range_mount_plate = LaserSensorShortRangeMountPlate()
    laser_sensor_short_range_mount_plate.export()





