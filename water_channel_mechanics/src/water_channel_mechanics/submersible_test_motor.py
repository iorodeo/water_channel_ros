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


SUBMERSIBLE_TEST_MOTOR_PARAMETERS = {
    'shaft_diameter': 1.0,
    'body_diameter': 2.5,
    'shaft_length': 24,
    'show_origin': False,
    }

def get_parameters():
    return copy.deepcopy(SUBMERSIBLE_TEST_MOTOR_PARAMETERS)

class SubmersibleTestMotor(csg.Union):
    def __init__(self):
        super(SubmersibleTestMotor, self).__init__()
        self.parameters = SUBMERSIBLE_TEST_MOTOR_PARAMETERS
        self.__make_submersible_test_motor()
        self.__make_origin()

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_submersible_test_motor(self):
        cylinder1 = fso.Cylinder(
        self.add_obj(submersible_test_motor)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    submersible_test_motor = SubmersibleTestMotor()
    submersible_test_motor.export()





