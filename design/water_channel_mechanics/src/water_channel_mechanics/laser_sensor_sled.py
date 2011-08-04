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
import laser_sensor_long_range_mount_plate_bottom
import laser_sensor_long_range_mount_plate_top
import laser_sensor_long_range

LASER_SENSOR_SLED_PARAMETERS = {
    'color': [0.7,0.7,0.7,1.0],
    'x': 4,
    'y': 50,
    'z': 2,
    'plate_thickness': 0.5,
    'show_origin': False,
    }

def get_parameters():
    return copy.deepcopy(LASER_SENSOR_SLED_PARAMETERS)

class LaserSensorSled(csg.Union):
    def __init__(self):
        super(LaserSensorSled, self).__init__()
        self.parameters = LASER_SENSOR_SLED_PARAMETERS
        self.__make_pillowblocks_and_plates()
        self.__make_y_beam()
        self.__make_laser_sensor_mount_plates()
        self.__make_laser_sensor_long_range()
        self.__make_origin()

    def __make_pillowblocks_and_plates(self):
        pb = pillowblock.PillowBlock()
        self.pb_parameters = pb.get_parameters()
        pmp = pillowblock_mount_plate.PillowblockMountPlate()
        self.pmp_parameters = pillowblock_mount_plate.get_parameters()

        pb_tx = 0
        self.wc_parameters = water_channel.get_parameters()
        pb_ty = self.wc_parameters['rail_rail_distance']/2
        pb_ay = [-pb_ty,pb_ty]
        pb_tz = 0

        self.pmp_tz = self.pb_parameters['y']/2 + self.pmp_parameters['z']/2
        pmp.translate([0,0,self.pmp_tz])
        pb_and_pmp = pb | pmp

        pbs_and_pmps = po.LinearArray(pb_and_pmp,x=pb_tx,y=pb_ay,z=pb_tz)
        self.add_obj(pbs_and_pmps)

    def __make_y_beam(self):
        x = self.parameters['x']
        y = self.parameters['y']
        z = self.parameters['z']
        y_beam = t_slotted.Extrusion(x=x,y=y,z=z)
        y_beam_tx = 0
        y_beam_ty = 0
        self.y_beam_tz = self.pmp_tz + self.pmp_parameters['z']/2 + z/2

        bn = t_slotted.LBracket(x=-2,y=2,z=2,extrusion_axis=[0,1,0])
        bn_tx = -self.parameters['x']/2
        b_ty = self.wc_parameters['rail_rail_distance']/2 - 1
        b_tz = -self.parameters['z']/2
        bsn = po.LinearArray(bn,x=bn_tx,y=[-b_ty,b_ty],z=b_tz)

        bp = t_slotted.LBracket(x=2,y=2,z=2,extrusion_axis=[0,1,0])
        bp_tx = self.parameters['x']/2
        bsp = po.LinearArray(bp,x=bp_tx,y=[-b_ty,b_ty],z=b_tz)
        y_beam |= [bsn,bsp]

        y_beam.translate([y_beam_tx,y_beam_ty,self.y_beam_tz])
        y_beam.set_color(self.parameters['color'],recursive=True)
        self.add_obj(y_beam)

    def __make_laser_sensor_mount_plates(self):
        bottom_plate = laser_sensor_long_range_mount_plate_bottom.LaserSensorLongRangeMountPlateBottom()
        self.bp_parameters = bottom_plate.get_parameters()
        self.bottom_plate_tz = self.y_beam_tz + self.bp_parameters['z']/2 + self.parameters['z']/2
        bottom_plate.translate([0,0,self.bottom_plate_tz])

        top_plate = laser_sensor_long_range_mount_plate_top.LaserSensorLongRangeMountPlateTop()
        self.tp_parameters = top_plate.get_parameters()
        self.top_plate_tz = self.bottom_plate_tz + self.bp_parameters['z']/2 + self.tp_parameters['z']/2
        top_plate.translate([0,0,self.top_plate_tz])

        self.add_obj([bottom_plate,top_plate])

    def __make_laser_sensor_long_range(self):
        laser_sensor = laser_sensor_long_range.LaserSensorLongRange()
        self.ls_parameters = laser_sensor.get_parameters()
        laser_sensor_tz = self.top_plate_tz + self.tp_parameters['z']/2 + self.ls_parameters['z']/2
        laser_sensor.translate([0,0,laser_sensor_tz])
        self.add_obj(laser_sensor)

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    laser_sensor_sled = LaserSensorSled()
    laser_sensor_sled.export()





