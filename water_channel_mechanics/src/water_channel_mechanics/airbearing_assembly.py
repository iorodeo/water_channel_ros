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
import cad.export.bom as bom

import airbearing
import airbearing_mount_plate
import slider_mount_plate
import submersible
import submersible_mount
import loadcell
import loadcell_mount_plate

AIRBEARING_ASSEMBLY_PARAMETERS = {
    'bearing_type': 'RAB6',
    'slide_travel': 4,
    'slider_mount_plate_thickness': 0.125,
    'y_beam_separation': 4,
    'x_beam_separation': 3,
    'cushion_thickness': 0.25,
    'submersible_mount_x_offset': 1.0625,
    'hole_diameter': 0.26,
    'color': [0.7,0.7,0.7,1.0],
    'show_origin': False,
    }

class AirbearingAssembly(csg.Union):
    def __init__(self):
        super(AirbearingAssembly, self).__init__()
        self.parameters = AIRBEARING_ASSEMBLY_PARAMETERS
        self.__make_airbearing()
        self.__make_airbearing_mount_plate()
        self.__make_slider_mount_plates()
        self.__make_y_beams()
        self.__make_cushions()
        self.__make_z_beams()
        self.__make_x_beams()
        self.__make_submersible_mount()
        self.__make_submersible_mount_beam()
        self.__make_submersible()
        self.__make_loadcell_mount_plate_lower()
        self.__make_loadcell()
        self.__make_loadcell_mount_plate_upper()
        self.__make_brackets()
        self.__make_sensor_target()

        self_tz = -self.ab_parameters['carriage_height']/2 - self.abmp_parameters['z']
        self.translate([0,0,self_tz])
        self.__make_origin()

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_airbearing(self):
        ab = airbearing.RAB(bearing_type=self.parameters['bearing_type'],slide_travel=self.parameters['slide_travel'])
        self.ab_parameters = ab.get_parameters()
        self.slider_holes_tx = (self.ab_parameters['slide_base_length'] + self.parameters['slide_travel'])/2 - self.ab_parameters['slide_screw_inset']
        self.add_obj(ab)

    def __make_airbearing_mount_plate(self):
        abmp = airbearing_mount_plate.AirbearingMountPlate()
        self.abmp_parameters = abmp.get_parameters()
        abmp_tz = self.ab_parameters['carriage_height']/2 + self.abmp_parameters['z']/2
        abmp.translate([0,0,abmp_tz])
        # abmp.translate([0,0,20])
        self.add_obj(abmp)

    def __make_slider_mount_plates(self):
        smp = slider_mount_plate.SliderMountPlate()
        self.smp_parameters = smp.get_parameters()
        smp_tx = self.slider_holes_tx
        smp_ty = 0
        smp_tz = -(self.ab_parameters['slide_height'] + self.smp_parameters['z'])/2
        smps = po.LinearArray(smp,x=[-smp_tx,smp_tx],y=smp_ty,z=smp_tz)
        smps.set_color(self.parameters['color'],recursive=True)
        self.add_obj(smps)

    def __make_y_beams(self):
        x = 1
        y = self.ab_parameters['slide_width']
        z = 1
        y_beam = t_slotted.Extrusion(x=x,y=y,z=z)
        y_beam_tx = self.slider_holes_tx
        y_beam_ty = 0
        y_beam_separation = -self.parameters['y_beam_separation']

        y_beams = po.LinearArray(y_beam,x=[-y_beam_tx,y_beam_tx],y=y_beam_ty,z=[0,y_beam_separation])
        self.y_beams_tz = -(self.ab_parameters['slide_height'] + z)/2 - self.smp_parameters['z']
        y_beams.translate([0,0,self.y_beams_tz])
        y_beams.set_color(self.parameters['color'],recursive=True)
        self.add_obj(y_beams)

    def __make_cushions(self):
        x = 1 + self.parameters['cushion_thickness']*2
        y = self.ab_parameters['slide_width'] + self.parameters['cushion_thickness']*2
        z = 0.25
        cushion = fso.Box(x=x,y=y,z=z)
        cutout_x = 3/8
        cutout_y = self.ab_parameters['slide_width'] + 0.01
        cutout_z = 0.5
        cutout = fso.Box(x=cutout_x,y=cutout_y,z=cutout_z)
        cushion -= cutout
        cushion.set_color([0.1,0.1,0.1],recursive=True)

        cushion_tx = self.slider_holes_tx
        cushion_ty = 0
        cushion_tz = -(self.ab_parameters['slide_height'] + 1)/2 - self.smp_parameters['z']
        cushions = po.LinearArray(cushion,x=[-cushion_tx,cushion_tx],y=cushion_ty,z=cushion_tz)
        self.add_obj(cushions)

    def __make_z_beams(self):
        x = 1
        y = 1
        z = self.parameters['y_beam_separation'] - 1
        z_beam = t_slotted.Extrusion(x=x,y=y,z=z)
        z_beam.set_color(self.parameters['color'])

        z_beam_tx = self.slider_holes_tx
        z_beam_ty = self.ab_parameters['slide_width']/2 - 0.5
        self.z_beam_tz = self.y_beams_tz - z/2 - 0.5
        z_beams = po.LinearArray(z_beam,x=[-z_beam_tx,z_beam_tx],y=[-z_beam_ty,z_beam_ty],z=self.z_beam_tz)
        self.add_obj(z_beams)

    def __make_x_beams(self):
        x = self.slider_holes_tx*2 - 1
        y = 1
        z = 1
        x_beam = t_slotted.Extrusion(x=x,y=y,z=z)
        x_beam.set_color(self.parameters['color'])

        x_beam_tx = 0
        x_beam_ty = self.ab_parameters['slide_width']/2 - 0.5
        self.x_beam_tz = self.y_beams_tz - self.parameters['y_beam_separation']
        x_beams = po.LinearArray(x_beam,x=x_beam_tx,y=[-x_beam_ty,x_beam_ty],z=[self.x_beam_tz,(self.x_beam_tz + self.parameters['x_beam_separation'])])
        self.add_obj(x_beams)

    def __make_submersible_mount(self):
        sm = submersible_mount.SubmersibleMount()
        self.sm_tz = self.x_beam_tz + self.parameters['x_beam_separation']/2
        sm.translate([0,0,self.sm_tz])
        self.add_obj(sm)

    def __make_submersible_mount_beam(self):
        x = 1
        y = self.ab_parameters['slide_width']
        z = 2
        smb = t_slotted.Extrusion(x=x,y=y,z=z)
        self.smb_tx = -self.parameters['submersible_mount_x_offset'] - x/2
        smb_ty = 0
        smb_tz = self.sm_tz
        smb.translate([self.smb_tx,smb_ty,smb_tz])
        smb.set_color(self.parameters['color'])
        self.add_obj(smb)

    def __make_submersible(self):
        sub = submersible.Submersible()
        self.sub_parameters = submersible.get_parameters()
        sub_tz = self.x_beam_tz - self.sub_parameters['shaft_length'] + self.parameters['x_beam_separation'] + 0.5
        sub.translate([0,0,sub_tz])
        self.add_obj(sub)

    def __make_loadcell_mount_plate_lower(self):
        lcmpl = loadcell_mount_plate.LoadcellMountPlate()
        self.lcmp_parameters = lcmpl.get_parameters()
        self.lcmpl_tx = -self.slider_holes_tx - 0.5 - self.lcmp_parameters['x']/2
        lcmpl_ty = 0
        self.lcmpl_tz = self.x_beam_tz
        lcmpl.translate([self.lcmpl_tx,lcmpl_ty,self.lcmpl_tz])
        self.add_obj(lcmpl)

    def __make_loadcell(self):
        lc = loadcell.Loadcell()
        self.lc_parameters = lc.get_parameters()
        self.lc_tx = self.lcmpl_tx - self.lcmp_parameters['x']/2 - self.lc_parameters['y']/2
        lc_ty = 0
        self.lc_tz = self.lcmpl_tz + self.lc_parameters['hole_x'][1]
        lc.translate([self.lc_tx,lc_ty,self.lc_tz])
        self.add_obj(lc)

    def __make_loadcell_mount_plate_upper(self):
        lcmpu = loadcell_mount_plate.LoadcellMountPlate()
        self.lcmpu_tx = self.lc_tx - self.lc_parameters['y']/2 - self.lcmp_parameters['x']/2
        lcmpu_ty = 0
        self.lcmpu_tz = self.lc_tz + self.lc_parameters['hole_x'][1]
        lcmpu.translate([self.lcmpu_tx,lcmpu_ty,self.lcmpu_tz])
        self.add_obj(lcmpu)

    def __make_brackets(self):
        # Front and back sets
        bracket_tx = 0
        bracket_ty = self.ab_parameters['slide_width']/2 - 1
        bracket_tz = (self.parameters['y_beam_separation'] - 1)/2
        bracket1 = t_slotted.LBracket(x=1,y=1,z=1,extrusion_axis=[1,0,0])
        bracket1.translate([bracket_tx,-bracket_ty,-bracket_tz])
        bracket2 = t_slotted.LBracket(x=1,y=-1,z=1,extrusion_axis=[1,0,0])
        bracket2.translate([bracket_tx,bracket_ty,-bracket_tz])
        bracket3 = t_slotted.LBracket(x=1,y=-1,z=-1,extrusion_axis=[1,0,0])
        bracket3.translate([bracket_tx,bracket_ty,bracket_tz])
        bracket4 = t_slotted.LBracket(x=1,y=1,z=-1,extrusion_axis=[1,0,0])
        bracket4.translate([bracket_tx,-bracket_ty,bracket_tz])
        brackets = bracket1 | [bracket2,bracket3,bracket4]
        brackets_tx = self.slider_holes_tx
        brackets_ty = 0
        brackets_tz = self.z_beam_tz
        brackets = po.LinearArray(brackets,x=[-brackets_tx,brackets_tx],y=brackets_ty,z=brackets_tz)
        brackets.set_color(self.parameters['color'],recursive=True)
        self.add_obj(brackets)

        # Left and right sets
        bracket_tx = (self.slider_holes_tx*2 - 1)/2
        bracket_ty = 0
        bracket_tz = (self.parameters['y_beam_separation'] - 2)/2
        bracket1 = t_slotted.LBracket(x=1,y=1,z=1,extrusion_axis=[0,1,0])
        bracket1.translate([-bracket_tx,bracket_ty,-bracket_tz])
        bracket2 = t_slotted.LBracket(x=1,y=1,z=-1,extrusion_axis=[0,1,0])
        bracket2.translate([-bracket_tx,bracket_ty,bracket_tz])
        bracket3 = t_slotted.LBracket(x=-1,y=1,z=-1,extrusion_axis=[0,1,0])
        bracket3.translate([bracket_tx,bracket_ty,bracket_tz])
        bracket4 = t_slotted.LBracket(x=-1,y=1,z=1,extrusion_axis=[0,1,0])
        bracket4.translate([bracket_tx,bracket_ty,-bracket_tz])
        brackets = bracket1 | [bracket2,bracket3,bracket4]
        brackets_tx = 0
        brackets_ty = self.ab_parameters['slide_width']/2 - 0.5
        brackets_tz = self.z_beam_tz - 0.5
        brackets = po.LinearArray(brackets,x=brackets_tx,y=[-brackets_ty,brackets_ty],z=brackets_tz)
        brackets.set_color(self.parameters['color'],recursive=True)
        self.add_obj(brackets)

        # Bottom set
        bracket_tx = (self.slider_holes_tx*2 - 1)/2
        bracket_ty = self.ab_parameters['slide_width']/2 - 1
        bracket_tz = 0
        bracket1 = t_slotted.LBracket(x=1,y=1,z=1,extrusion_axis=[0,0,1])
        bracket1.translate([-bracket_tx,-bracket_ty,bracket_tz])
        bracket2 = t_slotted.LBracket(x=1,y=-1,z=1,extrusion_axis=[0,0,1])
        bracket2.translate([-bracket_tx,bracket_ty,bracket_tz])
        bracket3 = t_slotted.LBracket(x=-1,y=-1,z=1,extrusion_axis=[0,0,1])
        bracket3.translate([bracket_tx,bracket_ty,bracket_tz])
        bracket4 = t_slotted.LBracket(x=-1,y=1,z=1,extrusion_axis=[0,0,1])
        bracket4.translate([bracket_tx,-bracket_ty,bracket_tz])
        brackets = bracket1 | [bracket2,bracket3,bracket4]
        brackets_tx = 0
        brackets_ty = 0
        brackets_tz = self.x_beam_tz
        brackets.translate([brackets_tx,brackets_ty,brackets_tz])
        brackets.set_color(self.parameters['color'],recursive=True)
        self.add_obj(brackets)

        # Submersible mount beam brackets
        bracket_tx = 0.5
        bracket_ty = 0
        bracket_tz = (self.parameters['y_beam_separation'] - 2)/2
        bracket1 = t_slotted.LBracket(x=1,y=1,z=1,extrusion_axis=[0,1,0])
        bracket1.translate([bracket_tx,bracket_ty,-bracket_tz])
        bracket2 = t_slotted.LBracket(x=1,y=1,z=-1,extrusion_axis=[0,1,0])
        bracket2.translate([bracket_tx,bracket_ty,bracket_tz])
        bracket3 = t_slotted.LBracket(x=-1,y=1,z=-1,extrusion_axis=[0,1,0])
        bracket3.translate([-bracket_tx,bracket_ty,bracket_tz])
        bracket4 = t_slotted.LBracket(x=-1,y=1,z=1,extrusion_axis=[0,1,0])
        bracket4.translate([-bracket_tx,bracket_ty,-bracket_tz])
        brackets = bracket1 | [bracket2,bracket3,bracket4]
        brackets_tx = 0
        brackets_ty = self.ab_parameters['slide_width']/2 - 0.5
        brackets_tz = self.z_beam_tz - 0.5
        brackets = po.LinearArray(brackets,x=brackets_tx,y=[-brackets_ty,brackets_ty],z=brackets_tz)
        brackets.translate([self.smb_tx,0,0])
        brackets.set_color(self.parameters['color'],recursive=True)
        self.add_obj(brackets)

    def __make_sensor_target(self):
        x = 0.236
        y = self.ab_parameters['slide_width']
        z = self.parameters['y_beam_separation'] + 0.25
        sensor_target = fso.Box(x=x,y=y,z=z)
        hole_r = 0.125
        hole_l = x*2
        hole = fso.Cylinder(r=hole_r,l=hole_l)
        hole.rotate(angle=-math.pi/2,axis=[0,1,0])
        hole_x = 0
        hole_y = y/2 - 0.5
        hole_az = [-1,0,1]
        holes = po.LinearArray(hole,x=hole_x,y=[-hole_y,hole_y],z=hole_az)
        sensor_target -= holes
        sensor_target_tx = self.slider_holes_tx + x/2 + 0.5
        sensor_target_ty = 0
        sensor_target_tz = self.z_beam_tz - 0.375
        sensor_target.translate([sensor_target_tx,sensor_target_ty,sensor_target_tz])
        sensor_target.set_color([0.98,0.98,0.98,1.0],recursive=True)
        self.add_obj(sensor_target)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    airbearing_assembly = AirbearingAssembly()
    airbearing_assembly.export()





