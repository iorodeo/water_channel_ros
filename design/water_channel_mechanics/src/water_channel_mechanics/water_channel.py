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


WATER_CHANNEL_PARAMETERS = {
    'rail_diameter': 1.5,
    'rail_rail_distance': 48.75,
    'rail_length': 1200,
    'rail_tank_distance': 1.75,
    'rail_color': [0.5,0.5,0.5,1.0],
    'tank_length': 1200,
    'tank_thickness': 0.5,
    'tank_color': [0.5,0.5,0.5,0.25],
    'show_tank': False,
    'channel_depth': 23.5,
    'channel_width': 43.25,
    'water_depth': 17.25,
    'water_color': [0.0,0.0,1.0,0.25],
    'show_water': False,
    'show_origin': False,
    }

def get_parameters():
    return copy.deepcopy(WATER_CHANNEL_PARAMETERS)

class WaterChannel(csg.Union):
    def __init__(self):
        super(WaterChannel, self).__init__()
        self.parameters = WATER_CHANNEL_PARAMETERS
        self.__make_rails()
        self.__make_tank()
        self.__make_water()
        # self.show_origin()
        self.__make_origin()

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_rails(self):
        rail_radius = self.parameters['rail_diameter']/2
        rail_rail_distance = self.parameters['rail_rail_distance']
        rail_length = self.parameters['rail_length']
        rail_color = self.parameters['rail_color']

        rail = fso.Cylinder(r=rail_radius,l=rail_length)
        rail.rotate(angle=math.pi/2,axis=[0,1,0])

        rails = po.LinearArray(rail,x=[0],y=[-rail_rail_distance/2,rail_rail_distance/2],z=[0])
        rails.set_color(rail_color,recursive=True)
        self.add_obj(rails)

    def __make_tank(self):
        tank_length = self.parameters['tank_length']
        tank_thickness = self.parameters['tank_thickness']
        channel_depth = self.parameters['channel_depth']
        channel_width = self.parameters['channel_width']
        rail_tank_distance = self.parameters['rail_tank_distance']
        tank_color = self.parameters['tank_color']
        show_tank = self.parameters['show_tank']

        channel = fso.Box(x=tank_length*1.1,y=channel_width,z=channel_depth*2)
        tank = fso.Box(x=tank_length,y=channel_width+tank_thickness*2,z=channel_depth*2+tank_thickness*2)
        tank -= channel

        tank_half = fso.Box([tank_length*1.1,(channel_width+tank_thickness*2)*1.1,(channel_depth+tank_thickness)*2])
        tank_half.translate([0,0,channel_depth+tank_thickness])
        tank -= tank_half
        tank.translate([0,0,-rail_tank_distance])
        tank.set_color(tank_color,recursive=True)

        if show_tank:
            self.add_obj(tank)

    def __make_water(self):
        tank_length = self.parameters['tank_length']
        channel_depth = self.parameters['channel_depth']
        channel_width = self.parameters['channel_width']
        rail_tank_distance = self.parameters['rail_tank_distance']
        water_depth = self.parameters['water_depth']
        water_color = self.parameters['water_color']
        show_water = self.parameters['show_water']
        water = fso.Box(x=tank_length,y=channel_width,z=water_depth)
        water.translate([0,0,-water_depth/2-(channel_depth-water_depth)-rail_tank_distance])
        water.set_color(water_color)
        if show_water:
            self.add_obj(water)

    # def show_origin(show=False,recursive=False):
    #     pass

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)

if __name__ == "__main__":
    water_channel = WaterChannel()
    water_channel.export()





