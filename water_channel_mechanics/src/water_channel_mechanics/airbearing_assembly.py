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

import airbearing

AIRBEARING_ASSEMBLY_PARAMETERS = {
    'bearing_type': 'RAB6',
    'slide_travel': 4,
    'hole_diameter': 0.26,
    'color': [0.7,0.7,0.7,1.0],
    'show_origin': True,
    }

class AirbearingAssembly(csg.Union):
    def __init__(self):
        super(AirbearingAssembly, self).__init__()
        self.parameters = AIRBEARING_ASSEMBLY_PARAMETERS
        self.__make_airbearing()
        self.__make_airbearing_mount_plate()
        self.__make_slider_mount_plates()
        self.__make_origin()

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_airbearing(self):
        ab = airbearing.RAB(bearing_type=self.parameters['bearing_type'],slide_travel=self.parameters['slide_travel'])
        self.ab_parameters = ab.get_parameters()
        self.add_obj(ab)

    def __make_airbearing_mount_plate(self):
        x = self.ab_parameters['carriage_length']
        y = self.ab_parameters['carriage_width']
        z = 0.5
        abmp = fso.Box(x=x,y=y,z=z)

        # Add airbearing mount holes
        hole_diameter = self.parameters['hole_diameter']
        hole = fso.Cylinder(r=hole_diameter/2,l=z*2)
        h_x = self.ab_parameters['carriage_screw_dL']/2
        h_y = self.ab_parameters['carriage_screw_dW']/2
        holes = po.LinearArray(hole,x=[-h_x,h_x],y=[-h_y,h_y],z=0)
        abmp -= holes

        # Add y_beam mount counterbore holes
        cb_diameter = 7/16
        cb_depth = 0.25
        cb = fso.Cylinder(r=cb_diameter/2,l=z)
        cb.translate([0,0,(-z + cb_depth)])
        cbh = cb | hole
        cbhs = po.LinearArray(cbh,x=[-2.5,0,2.5],y=[-1.5,1.5],z=0)
        abmp -= cbhs

        abmp_tz = self.ab_parameters['carriage_height']/2 + z/2
        abmp.translate([0,0,abmp_tz])
        # abmp.translate([0,0,20])
        abmp.set_color(self.parameters['color'],recursive=True)
        self.add_obj(abmp)

    def __make_slider_mount_plates(self):
        x = 1
        y = self.ab_parameters['slide_width']
        z = 0.125
        smp = fso.Box(x=x,y=y,z=z)

        hole = fso.Cylinder(r=self.parameters['hole_diameter']/2,l=z*2)
        hole_ty = self.ab_parameters['slide_screw_dW']/2
        holes = po.LinearArray(hole,x=0,y=[-hole_ty,hole_ty],z=0)
        smp -= holes
        smp_tx = (self.ab_parameters['slide_base_length'] + self.parameters['slide_travel'])/2 - self.ab_parameters['slide_screw_inset']
        smp_ty = 0
        smp_tz = -(self.ab_parameters['slide_height'] + z)/2
        smps = po.LinearArray(smp,x=[-smp_tx,smp_tx],y=smp_ty,z=smp_tz)
        smps.set_color(self.parameters['color'],recursive=True)
        self.add_obj(smps)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    airbearing_assembly = AirbearingAssembly()
    airbearing_assembly.export()





