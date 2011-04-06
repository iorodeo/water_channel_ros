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

import loadcell

LOADCELL_MOUNT_PLATE_PARAMETERS = {
    'color': [0.7,0.7,0.7,1.0],
    'x': 0.125,
    'y': 6.0,
    'z': 1.0,
    'hole_r_big': 0.13,
    'hole_y_big': 2.5,
    'hole_r_loadcell': 0.0625,
    'hole_l': 1.0,
    'show_origin': False,
    }

def get_parameters():
    return copy.deepcopy(LOADCELL_MOUNT_PLATE_PARAMETERS)

class LoadcellMountPlate(csg.Union):
    def __init__(self):
        super(LoadcellMountPlate, self).__init__()
        self.parameters = LOADCELL_MOUNT_PLATE_PARAMETERS
        self.loadcell_parameters = loadcell.get_parameters()
        self.__make_loadcell_mount_plate()
        self.__make_origin()

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_loadcell_mount_plate(self):
        length = self.parameters['x']
        width = self.parameters['y']
        height = self.parameters['z']
        loadcell_mount_plate = fso.Box(x=length,y=width,z=height)

        hole_l = self.parameters['hole_l']

        # Loadcell mount holes
        hole_r_loadcell = self.parameters['hole_r_loadcell']
        base_hole = fso.Cylinder(r=hole_r_loadcell,l=hole_l)
        base_hole.rotate(angle=math.pi/2,axis=[0,1,0])

        hole_y = self.loadcell_parameters['hole_z']
        holes = po.LinearArray(base_hole,0,hole_y,0)
        loadcell_mount_plate -= holes

        # T-slotted bracket mount holes
        hole_r_big = self.parameters['hole_r_big']
        base_hole = fso.Cylinder(r=hole_r_big,l=hole_l)
        base_hole.rotate(angle=math.pi/2,axis=[0,1,0])

        hole_y = self.parameters['hole_y_big']
        holes = po.LinearArray(base_hole,0,[-hole_y,hole_y],0)
        loadcell_mount_plate -= holes

        loadcell_mount_plate.set_color(self.parameters['color'],recursive=True)
        self.add_obj(loadcell_mount_plate)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    loadcell_mount_plate = LoadcellMountPlate()
    loadcell_mount_plate.export()





