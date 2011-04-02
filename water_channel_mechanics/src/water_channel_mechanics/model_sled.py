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
import cad_library.t_slotted as t_slotted

import water_channel
import pillowblock
import pillowblock_mount_plate

MODEL_SLED_PARAMETERS = {
    'color': [0.7,0.7,0.7,1.0],
    'x': 40,
    'y': 50,
    'z': 2,
    'beam_width': 4,
    'show_origin': True,
    }

def get_parameters():
    return copy.deepcopy(MODEL_SLED_PARAMETERS)

class ModelSled(csg.Union):
    def __init__(self):
        super(ModelSled, self).__init__()
        self.parameters = MODEL_SLED_PARAMETERS
        self.__make_pillowblocks_and_plates()
        self.__make_y_beam()
        self.__make_x_beams()
        self.__make_brackets()
        self.__make_origin()

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_pillowblocks_and_plates(self):
        pb = pillowblock.PillowBlock()
        self.pb_parameters = pb.get_parameters()
        pmp = pillowblock_mount_plate.PillowblockMountPlate()
        self.pmp_parameters = pillowblock_mount_plate.get_parameters()

        self.pb_tx = self.parameters['x']/2 - self.parameters['beam_width']/2
        pb_ax = [-self.pb_tx,self.pb_tx]
        self.wc_parameters = water_channel.get_parameters()
        pb_ty = self.wc_parameters['rail_rail_distance']/2
        pb_ay = [-pb_ty,pb_ty]
        pb_az = [0]

        self.pmp_tz = self.pb_parameters['y']/2 + self.pmp_parameters['z']/2
        pmp.translate([0,0,self.pmp_tz])
        pb_and_pmp = pb | pmp

        pbs_and_pmps = po.LinearArray(pb_and_pmp,x=pb_ax,y=pb_ay,z=pb_az)
        self.add_obj(pbs_and_pmps)

    def __make_y_beam(self):
        beam_width = self.parameters['beam_width']
        x = self.parameters['x'] - beam_width*2
        y = beam_width
        z = self.parameters['z']
        y_beam = t_slotted.Extrusion(x=x,y=y,z=z)
        # self.y_beam_ty = -0.375/2
        self.y_beam_ty = 0
        self.y_beam_tz = self.pmp_tz + self.pmp_parameters['z']/2 + z/2

        b1 = t_slotted.LBracket(x=2,y=2,z=2,extrusion_axis=[0,0,1])
        b1.translate([-x/2,beam_width/2,0])
        b2 = t_slotted.LBracket(x=-2,y=2,z=2,extrusion_axis=[0,0,1])
        b2.translate([x/2,beam_width/2,0])
        b3 = t_slotted.LBracket(x=2,y=-2,z=2,extrusion_axis=[0,0,1])
        b3.translate([-x/2,-beam_width/2,0])
        b4 = t_slotted.LBracket(x=-2,y=-2,z=2,extrusion_axis=[0,0,1])
        b4.translate([x/2,-beam_width/2,0])
        y_beam |= [b1,b2,b3,b4]

        y_beam.translate([0,self.y_beam_ty,self.y_beam_tz])
        y_beam.set_color(self.parameters['color'],recursive=True)
        self.add_obj(y_beam)

    def __make_x_beams(self):
        x = self.parameters['beam_width']
        y = self.parameters['y']
        z = self.parameters['z']
        x_beam = t_slotted.Extrusion(x=x,y=y,z=z)
        x_beam_tx = self.parameters['x']/2 - self.parameters['beam_width']/2
        x_beam_ty = 0
        x_beam_tz = self.y_beam_tz

        bn = t_slotted.LBracket(x=-2,y=2,z=2,extrusion_axis=[0,1,0])
        bn_tx = -self.parameters['beam_width']/2
        b_ty = self.wc_parameters['rail_rail_distance']/2 - 1
        b_tz = -self.parameters['z']/2
        bsn = po.LinearArray(bn,x=bn_tx,y=[-b_ty,b_ty],z=b_tz)

        bp = t_slotted.LBracket(x=2,y=2,z=2,extrusion_axis=[0,1,0])
        bp_tx = self.parameters['beam_width']/2
        bsp = po.LinearArray(bp,x=bp_tx,y=[-b_ty,b_ty],z=b_tz)
        x_beam |= [bsn,bsp]

        x_beams = po.LinearArray(x_beam,x=[-x_beam_tx,x_beam_tx],y=x_beam_ty,z=x_beam_tz)
        x_beams.set_color(self.parameters['color'],recursive=True)
        self.add_obj(x_beams)

    def __make_brackets(self):
        b = t_slotted.LBracket(x=-1,y=2,z=1,extrusion_axis=[0,1,0])
        b_x = self.parameters['x']/2 + 2
        b_y = self.wc_parameters['rail_rail_distance']/2 + 1
        b_z = self.y_beam_tz - self.parameters['z']/2
        bs = po.LinearArray(b,x=b_x,y=[-b_y,b_y],z=b_z)
        bs.set_color(self.parameters['color'],recursive=True)
        self.add_obj(bs)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    model_sled = ModelSled()
    model_sled.export()





