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

import loadcell

LOADCELL_MOUNT_PLATE_PARAMETERS = {
    'color': [0.7,0.7,0.7,1.0],
    'x': 6.0,
    'y': 1.0,
    'z': 0.125,
    'hole_r_big': 0.13,
    'hole_x_big': 2.5,
    'hole_r_loadcell': 0.0625,
    'hole_l': 1.0,
    'show_origin': False,
    }

def get_parameters():
    return copy.deepcopy(LOADCELL_MOUNT_PLATE_PARAMETERS)

class LoadcellMountPlate(csg.Difference):
    def __init__(self):
        super(LoadcellMountPlate, self).__init__()
        self.parameters = LOADCELL_MOUNT_PLATE_PARAMETERS
        self.loadcell_parameters = loadcell.get_parameters()
        self.__make_loadcell_mount_plate()
        self.__make_holes()
        self.__set_bom()
        self.__make_origin()
        self.set_color(self.parameters['color'],recursive=True)

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_loadcell_mount_plate(self):
        length = self.parameters['x']
        width = self.parameters['y']
        height = self.parameters['z']
        loadcell_mount_plate = fso.Box(x=length,y=width,z=height)
        self.add_obj(loadcell_mount_plate)

    def __make_holes(self):
        hole_l = self.parameters['hole_l']

        # Loadcell mount holes
        hole_r_loadcell = self.parameters['hole_r_loadcell']
        base_hole = fso.Cylinder(r=hole_r_loadcell,l=hole_l)

        hole_x = self.loadcell_parameters['hole_z']
        holes = po.LinearArray(base_hole,hole_x,0,0)
        self.add_obj(holes)

        # T-slotted bracket mount holes
        hole_r_big = self.parameters['hole_r_big']
        base_hole = fso.Cylinder(r=hole_r_big,l=hole_l)

        hole_x = self.parameters['hole_x_big']
        holes = po.LinearArray(base_hole,[-hole_x,hole_x],0,0)
        self.add_obj(holes)

    def __set_bom(self):
        scale = self.get_scale()
        BOM = bom.BOMObject()
        BOM.set_parameter('name','loadcell_mount_plate')
        BOM.set_parameter('description','Mounts loadcell to t_slotted beams')
        BOM.set_parameter('dimensions','x: {x:0.3f}, y: {y:0.3f}, z: {z:0.3f}'.format(x=self.parameters['x']*scale[0],y=self.parameters['y']*scale[1],z=self.parameters['z']*scale[2]))
        BOM.set_parameter('vendor','?')
        BOM.set_parameter('part number','?')
        BOM.set_parameter('cost',0)
        self.set_object_parameter('bom',BOM)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    loadcell_mount_plate = LoadcellMountPlate()
    loadcell_mount_plate.set_object_parameter('slice',True)
    loadcell_mount_plate.export()





