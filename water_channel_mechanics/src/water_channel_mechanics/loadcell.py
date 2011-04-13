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


LOADCELL_PARAMETERS = {
    'x': 2.760,
    'y': 0.866,
    'z': 0.590,
    'profile': 'loadcell/loadcell.dxf',
    'color': [1.0,1.0,0.0,1.0],
    'hole_x': (-1.142,1.142),
    'hole_z': (-0.138,0.138),
    'hole_r': 0.06,
    'hole_l': 4,
    }

def get_parameters():
    return copy.deepcopy(LOADCELL_PARAMETERS)

class Loadcell(csg.Difference):
    def __init__(self):
        super(Loadcell, self).__init__()
        self.parameters = LOADCELL_PARAMETERS
        self.__make_loadcell()
        self.__make_holes()
        self.__set_bom()
        self.rotate(angle=math.pi/2,axis=[1,0,0])
        self.rotate(angle=math.pi/2,axis=[0,1,0])
        self.set_color([1,1,0],recursive=True)

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_loadcell(self):
        profile = self.parameters['profile']
        l = self.parameters['z']
        loadcell = fso.Extrusion(profile=profile,l=l)
        self.add_obj(loadcell)

    def __make_holes(self):
        hole_r = self.parameters['hole_r']
        hole_l = self.parameters['hole_l']
        hole_x = self.parameters['hole_x']
        hole_z = self.parameters['hole_z']
        hole_list = []
        base_hole = fso.Cylinder(r=hole_r,l=hole_l)
        base_x_hole = base_hole.copy()
        base_x_hole.rotate(angle=-math.pi/2,axis=[1,0,0])
        for z in hole_z:
            for x in hole_x:
                hole = base_x_hole.copy()
                hole.translate([x,0,z])
                hole_list.append(hole)
        self.add_obj(hole_list)

    def __set_bom(self):
        scale = self.get_scale()
        BOM = bom.BOMObject()
        BOM.set_parameter('name','loadcell')
        BOM.set_parameter('description','Low Single PT Load Cell')
        BOM.set_parameter('dimensions','2Kg')
        BOM.set_parameter('vendor','TransducerTechniques')
        BOM.set_parameter('part number','LSP-2')
        BOM.set_parameter('cost',110)
        self.set_object_parameter('bom',BOM)


if __name__ == "__main__":
    import cad_library.origin as origin
    o = origin.Origin()
    loadcell = Loadcell()
    # loadcell = loadcell | o
    loadcell.export()





