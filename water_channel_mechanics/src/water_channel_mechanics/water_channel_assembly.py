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
import submersible_sled
import motorized_sled

WATER_CHANNEL_ASSEMBLY_PARAMETERS = {
    'show_origin': False,
    }

class WaterChannelAssembly(csg.Union):
    def __init__(self):
        super(WaterChannelAssembly, self).__init__()
        self.parameters = WATER_CHANNEL_ASSEMBLY_PARAMETERS
        self.__make_water_channel()
        self.__make_sleds()
        self.__make_origin()

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_water_channel(self):
        wc = water_channel.WaterChannel()
        self.add_obj(wc)

    def __make_sleds(self):
        sled_submersible = submersible_sled.SubmersibleSled()
        sled_motorized = motorized_sled.MotorizedSled()
        sled_motorized.translate([(sled_submersible.pb_tx + sled_motorized.pb_tx + 9),0,0])
        sleds = sled_submersible | sled_motorized
        # sleds.translate([100,0,0])
        self.add_obj(sleds)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    water_channel_assembly = WaterChannelAssembly()
    water_channel_assembly.export()





