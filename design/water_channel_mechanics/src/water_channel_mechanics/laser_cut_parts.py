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

import laser_sensor_long_range_mount_plate_bottom
import laser_sensor_long_range_mount_plate_top
import laser_sensor_long_range_target_plate
import laser_sensor_short_range_mount_plate
import laser_sensor_short_range_target_plate


LASER_CUT_PARTS_PARAMETERS = {
    'offset': 7.5,
    }

def get_parameters():
    return copy.deepcopy(LASER_CUT_PARTS_PARAMETERS)

class LaserCutParts(csg.Union):
    def __init__(self):
        super(LaserCutParts, self).__init__()
        self.parameters = LASER_CUT_PARTS_PARAMETERS
        self.__make_plates()
        self.__make_reference_cube()
        self.set_object_parameter('slice',True)

    def __make_plates(self):
        lrmb = laser_sensor_long_range_mount_plate_bottom.LaserSensorLongRangeMountPlateBottom()
        self.lrmb_parameters = lrmb.get_parameters()
        lrmb.translate([-self.parameters['offset'],0,0])

        lrmt = laser_sensor_long_range_mount_plate_top.LaserSensorLongRangeMountPlateTop()
        self.lrmt_parameters = lrmt.get_parameters()
        lrmt.translate([self.parameters['offset'],0,0])

        lrt = laser_sensor_long_range_target_plate.LaserSensorLongRangeTargetPlate()
        self.lrt_parameters = lrt.get_parameters()

        srm = laser_sensor_short_range_mount_plate.LaserSensorShortRangeMountPlate()
        self.srm_parameters = srm.get_parameters()
        srm.translate([0,self.parameters['offset'],0])

        srt = laser_sensor_short_range_target_plate.LaserSensorShortRangeTargetPlate()
        self.srt_parameters = srt.get_parameters()
        srt.translate([0,-self.parameters['offset'],0])

        self.add_obj([lrmb,lrmt,lrt,srm,srt])

    def __make_reference_cube(self):
        rc = fso.Box(x=1,y=1,z=1)
        rc.translate([self.parameters['offset'],self.parameters['offset'],0])

        self.add_obj(rc)

    def get_parameters(self):
        return copy.deepcopy(self.parameters)


if __name__ == "__main__":
    laser_cut_parts = LaserCutParts()
    laser_cut_parts.export()





