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

import airbearing

SLIDER_MOUNT_PLATE_PARAMETERS = {
    'bearing_type': 'RAB6',
    'slide_travel': 4,
    'color': [0.7,0.7,0.7,1.0],
    'x': 1,
    'z': 0.125,
    'hole_diameter': 0.26,
    'show_origin': False,
    }

def get_parameters():
    return copy.deepcopy(SLIDER_MOUNT_PLATE_PARAMETERS)

class SliderMountPlate(csg.Difference):
    def __init__(self):
        super(SliderMountPlate, self).__init__()
        self.parameters = SLIDER_MOUNT_PLATE_PARAMETERS
        ab = airbearing.RAB(bearing_type=self.parameters['bearing_type'],slide_travel=self.parameters['slide_travel'])
        self.ab_parameters = ab.get_parameters()
        self.__make_slider_mount_plate()
        self.__make_holes()
        self.__set_bom()
        self.__make_origin()
        self.set_color(self.parameters['color'],recursive=True)

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_slider_mount_plate(self):
        x = self.parameters['x']
        y = self.ab_parameters['slide_width']
        self.parameters['y'] = y
        z = self.parameters['z']
        smp = fso.Box(x=x,y=y,z=z)

        self.add_obj(smp)

    def __make_holes(self):
        hole = fso.Cylinder(r=self.parameters['hole_diameter']/2,l=self.parameters['z']*2)
        hole_ty = self.ab_parameters['slide_screw_dW']/2
        holes = po.LinearArray(hole,x=0,y=[-hole_ty,hole_ty],z=0)
        self.add_obj(holes)

    def __set_bom(self):
        scale = self.get_scale()
        BOM = bom.BOMObject()
        BOM.set_parameter('name','slider_mount_plate')
        BOM.set_parameter('description','mounts t_slotted beams to air bearing slider')
        BOM.set_parameter('dimensions','x: {x:0.3f}, y: {y:0.3f}, z: {z:0.3f}'.format(x=self.parameters['x']*scale[0],y=self.parameters['y']*scale[1],z=self.parameters['z']*scale[2]))
        BOM.set_parameter('vender','?')
        BOM.set_parameter('part number','?')
        self.set_object_parameter('bom',BOM)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    slider_mount_plate = SliderMountPlate()
    slider_mount_plate.export()





