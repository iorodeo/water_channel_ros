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
import laser_sensor_long_range_target_plate

MOTORIZED_SLED_PARAMETERS = {
    'color': [0.7,0.7,0.7,1.0],
    'x': 40,
    'y': 53.375,
    'z': 2.5,
    'plate_thickness': 0.5,
    'show_origin': False,
    }

def get_parameters():
    return copy.deepcopy(MOTORIZED_SLED_PARAMETERS)

class MotorizedSled(csg.Union):
    def __init__(self):
        super(MotorizedSled, self).__init__()
        self.parameters = MOTORIZED_SLED_PARAMETERS
        self.__make_pillowblocks_and_plates()
        self.__make_horizontal_plate()
        self.__make_vertical_x_plates()
        self.__make_vertical_y_plates()
        self.__make_laser_sensor_long_range_target_plate()
        self.__make_origin()

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_pillowblocks_and_plates(self):
        pb = pillowblock.PillowBlock()
        self.pb_parameters = pb.get_parameters()
        pmp = pillowblock_mount_plate.PillowblockMountPlate()
        self.pmp_parameters = pillowblock_mount_plate.get_parameters()

        self.pb_tx = self.parameters['x']/2 - self.pmp_parameters['y']/2
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

        # Add sled-sled connection brackets and beams
        bracket = t_slotted.LBracket(x=1,y=2,z=1,extrusion_axis=[0,1,0])
        bracket2 = po.LinearArray(bracket,x=0,y=[-1,1],z=0)
        beam = t_slotted.Extrusion(x=1,y=5,z=1)
        beam.translate([-0.5,0,0.5])
        b2b = bracket2 | beam
        b2bs_ax = [-self.pb_tx - 4]
        b2bs_ay = [-pb_ty, pb_ty]
        b2bs_az = [self.pmp_tz + self.pmp_parameters['z']/2]
        b2bs = po.LinearArray(b2b,b2bs_ax,b2bs_ay,b2bs_az)
        b2bs.set_color(self.parameters['color'],recursive=True)
        self.add_obj(b2bs)

    def __make_horizontal_plate(self):
        x = self.parameters['x']
        y = self.parameters['y']
        z = self.parameters['plate_thickness']
        ph = fso.Box(x=x,y=y,z=z)
        self.ph_ty = -0.375/2
        self.ph_tz = self.pmp_tz + self.pmp_parameters['z']/2 + z/2
        ph.translate([0,self.ph_ty,self.ph_tz])
        ph.set_color(self.parameters['color'])
        self.add_obj(ph)

    def __make_vertical_x_plates(self):
        x = self.parameters['x']
        y = self.parameters['plate_thickness']
        z = self.parameters['z'] - self.parameters['plate_thickness']

        vxp = fso.Box(x=x,y=y,z=z)
        vxp_tx = 0
        vxp_ty = self.parameters['y']/2 - self.parameters['plate_thickness']/2
        vxp_tz = self.ph_tz + self.parameters['plate_thickness']/2 + z/2

        vxps = po.LinearArray(vxp,x=vxp_tx,y=[-vxp_ty,vxp_ty],z=vxp_tz)
        vxps.translate([0,self.ph_ty,0])
        vxps.set_color(self.parameters['color'],recursive=True)
        self.add_obj(vxps)

    def __make_vertical_y_plates(self):
        x = self.parameters['plate_thickness']
        y = self.parameters['y']
        z = self.parameters['z'] - self.parameters['plate_thickness']

        vyp = fso.Box(x=x,y=y,z=z)
        self.vyp_tx = self.parameters['x']/2 - self.parameters['plate_thickness']/2
        vyp_ty = 0
        self.vyp_tz = self.ph_tz + self.parameters['plate_thickness']/2 + z/2

        vyps = po.LinearArray(vyp,x=[-self.vyp_tx,self.vyp_tx],y=vyp_ty,z=self.vyp_tz)
        vyps.translate([0,self.ph_ty,0])
        vyps.set_color(self.parameters['color'],recursive=True)
        self.add_obj(vyps)

    def __make_laser_sensor_long_range_target_plate(self):
        ltp = laser_sensor_long_range_target_plate.LaserSensorLongRangeTargetPlate()
        self.ltp_parameters = ltp.get_parameters()
        ltp.rotate(angle=math.pi/2,axis=[0,0,1])
        ltp.rotate(angle=-math.pi/2,axis=[0,1,0])
        ltp_tx = self.vyp_tx + self.parameters['plate_thickness']/2 + self.ltp_parameters['z']/2
        ltp_ty = 0
        ltp_tz = self.vyp_tz
        ltp.translate([ltp_tx,ltp_ty,ltp_tz])
        self.add_obj(ltp)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    motorized_sled = MotorizedSled()
    motorized_sled.export()





