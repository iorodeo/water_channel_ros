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


CUSHION_PARAMETERS = {
    'bearing_type': 'RAB6',
    'slide_travel': 4,
    'color': [0.1,0.1,0.1,1.0],
    'z': 0.25,
    'cutout_x': 3/8,
    'cutout_z': 0.5,
    'show_origin': False,
    }

def get_parameters():
    return copy.deepcopy(CUSHION_PARAMETERS)

class Cushion(csg.Difference):
    def __init__(self):
        super(Cushion, self).__init__()
        self.parameters = CUSHION_PARAMETERS
        ab = airbearing.RAB(bearing_type=self.parameters['bearing_type'],slide_travel=self.parameters['slide_travel'])
        self.ab_parameters = ab.get_parameters()
        self.__make_cushion()
        self.__make_cutout()
        self.__set_bom()
        self.__make_origin()
        self.set_color(self.parameters['color'],recursive=True)

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_cushion(self):
        x = 1 + self.parameters['z']*2
        self.parameters['x'] = x
        y = self.ab_parameters['slide_width'] + self.parameters['z']*2
        self.parameters['y'] = y
        z = self.parameters['z']
        cushion = fso.Box(x=x,y=y,z=z)

        self.add_obj(cushion)

    def __make_cutout(self):
        x = self.parameters['cutout_x']
        y = self.ab_parameters['slide_width'] + 0.1
        self.parameters['cutout_y'] = y
        z = self.parameters['cutout_z']
        cutout = fso.Box(x=x,y=y,z=z)

        self.add_obj(cutout)

    def __set_bom(self):
        scale = self.get_scale()
        BOM = bom.BOMObject()
        BOM.set_parameter('name','cushion')
        BOM.set_parameter('description','Protects the air bearing carriage')
        BOM.set_parameter('dimensions','x: {x:0.3f}, y: {y:0.3f}, z: {z:0.3f}'.format(x=self.parameters['x']*scale[0],y=self.parameters['y']*scale[1],z=self.parameters['z']*scale[2]))
        BOM.set_parameter('vendor','McMaster')
        BOM.set_parameter('part number','9013K153')
        BOM.set_parameter('cost',15.94)
        self.set_object_parameter('bom',BOM)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    cushion = Cushion()
    cushion.set_object_parameter('slice',True)
    # convert to mm
    cushion.set_scale(25.4)
    cushion.export()





