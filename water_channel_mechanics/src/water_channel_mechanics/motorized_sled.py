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

import water_channel
import pillowblock
import pillowblock_mount_plate

MOTORIZED_SLED_PARAMETERS = {
    'color': [0.7,0.7,0.7,1.0],
    'x': 40,
    'y': 53.375,
    'z': 2.5,
    'plate_thickness': 0.5,
    'show_origin': True,
    }

def get_parameters():
    return copy.deepcopy(MOTORIZED_SLED_PARAMETERS)

class MotorizedSled(csg.Union):
    def __init__(self):
        super(MotorizedSled, self).__init__()
        self.parameters = MOTORIZED_SLED_PARAMETERS
        self.__make_pillowblocks()
        self.__make_pillowblock_mount_plates()
        self.__make_horizontal_plate()
        self.__make_origin()

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_pillowblocks(self):
        pb = pillowblock.PillowBlock()
        self.pillowblock_parameters = pb.get_parameters()
        x = self.parameters['x']/2 - self.pillowblock_parameters['z']/2
        pillowblock_x = [-x,x]
        self.parameters['pillowblock_x'] = pillowblock_x
        self.wc_parameters = water_channel.get_parameters()
        y = self.wc_parameters['rail_rail_distance']
        pillowblock_y = [-y,y]
        self.parameters['pillowblock_y'] = pillowblock_y
        self.parameters['pillowblock_z'] = 0
        pillowblocks = po.LinearArray(pb,x=pillowblock_x,y=pillowblock_y,z=[0])
        self.add_obj(pillowblocks)

    def __make_pillowblock_mount_plates(self):
        pmp = pillowblock_mount_plate.PillowblockMountPlate()
        pmp_x = self.parameters['pillowblock_x']
        pmp_y = self.parameters['pillowblock_y']
        pmp_z = self.pillowblock_parameters['y']/2
        pillowblock_mount_plates = po.LinearArray(pmp,x=pmp_x,y=pmp_y,z=pmp_z)
        self.add_obj(pillowblock_mount_plates)

    def __make_horizontal_plate(self):
        x = self.parameters['x']
        y = self.parameters['y']
        z = self.parameters['plate_thickness']
    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    motorized_sled = MotorizedSled()
    motorized_sled.export()





