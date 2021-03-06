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

import pillowblock

PILLOWBLOCK_MOUNT_PLATE_PARAMETERS = {
    'color': [0.7,0.7,0.7,1.0],
    'y': 5.0,
    'z': 0.25,
    'hole_r': 0.1,
    'hole_l': 2.0,
    'show_origin': False,
    }

def get_parameters():
    return copy.deepcopy(PILLOWBLOCK_MOUNT_PLATE_PARAMETERS)

class PillowblockMountPlate(csg.Difference):
    def __init__(self):
        super(PillowblockMountPlate, self).__init__()
        self.parameters = PILLOWBLOCK_MOUNT_PLATE_PARAMETERS
        self.pillowblock_parameters = pillowblock.get_parameters()
        self.__make_pillowblock_mount_plate()
        self.__make_holes()
        self.__set_bom()
        self.__make_origin()
        self.set_color(self.parameters['color'],recursive=True)

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_pillowblock_mount_plate(self):
        x = self.pillowblock_parameters['z'] + 4
        self.parameters['x'] = x
        y = self.parameters['y']
        z = self.parameters['z']
        pillowblock_mount_plate = fso.Box(x=x,y=y,z=z)
        self.add_obj(pillowblock_mount_plate)

    def __make_holes(self):
        hole_r = self.parameters['hole_r']
        hole_l = self.parameters['hole_l']

        # Pillowblock mount holes
        hole_x = self.pillowblock_parameters['hole_z']
        hole_y = self.pillowblock_parameters['hole_x']
        base_hole = fso.Cylinder(r=hole_r,l=hole_l)
        holes = po.LinearArray(base_hole,hole_x,hole_y,[0])
        self.add_obj(holes)

        # T-slotted bracket mount holes
        hole_x = [-3.5,-2.5,2.5,3.5]
        hole_y = [-1.5,-0.5,0.5,1.5]
        holes = po.LinearArray(base_hole,hole_x,hole_y,[0])
        self.add_obj(holes)

        # Motorized sled mount holes
        hole_x = [-2.125,2.125]
        hole_y = [-2.3125,2.3125]
        holes = po.LinearArray(base_hole,hole_x,hole_y,[0])
        self.add_obj(holes)

        # Slots for motorized sled mount holes
        hole = fso.Cylinder(r=0.125,l=self.parameters['z']*2)
        holes = po.LinearArray(hole,[0],[-0.5,0.5],[0])
        slot = fso.Box(x=0.25,y=1,z=self.parameters['z']*2)
        slot = holes | slot
        slots = po.LinearArray(slot,[-2.125,2.125],[0],[0])
        self.add_obj(slots)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)

    def __set_bom(self):
        scale = self.get_scale()
        BOM = bom.BOMObject()
        BOM.set_parameter('name','pillowblock_mount_plate')
        BOM.set_parameter('description','Mounts pillowblocks to sleds')
        BOM.set_parameter('dimensions','x: {x:0.3f}, y: {y:0.3f}, z: {z:0.3f}'.format(x=self.parameters['x']*scale[0],y=self.parameters['y']*scale[1],z=self.parameters['z']*scale[2]))
        BOM.set_parameter('vendor','?')
        BOM.set_parameter('part number','?')
        self.set_object_parameter('bom',BOM)


if __name__ == "__main__":
    pillowblock_mount_plate = PillowblockMountPlate()
    pillowblock_mount_plate.set_object_parameter('slice',True)
    pillowblock_mount_plate.export()





