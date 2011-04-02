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
    'body_length': 6,
    'shaft_length': 24,
    'color': [0.2,0.2,0.2],
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
        body_diameter = self.parameters['body_diameter']
        cylinder1 = fso.Cylinder(r=body_diameter/2,l=body_diameter)
        cylinder1.rotate(angle=math.pi/2,axis=[0,1,0])
        c2_length = self.parameters['body_length'] - body_diameter
        cylinder2 = fso.Cylinder(r=body_diameter/2,l=c2_length)
        cylinder2.rotate(angle=math.pi/2,axis=[0,1,0])
        cylinder2.translate([body_diameter/2 + c2_length/2,0,0])
        sphere = fso.Sphere(r=body_diameter/2)
        sphere.translate([body_diameter/2 + c2_length,0,0])
        cylinder3 = fso.Cylinder(r=self.parameters['shaft_diameter']/2,l=self.parameters['shaft_length'])
        cylinder3.translate([0,0,self.parameters['shaft_length']/2])
        box = fso.Box(x=2,y=0.25,z=body_diameter/2 + 2)
        box.translate([0,0,-(body_diameter/2 + 2)/2])
        submersible_test_motor = cylinder1 | cylinder2 | sphere | cylinder3 | box
        submersible_test_motor.set_color(self.parameters['color'],recursive=True)
        self.add_obj(submersible_test_motor)

    def __make_origin(self):
        o = origin.Origin(mag=10)
        if self.parameters['show_origin']:
            self.add_obj(o)


if __name__ == "__main__":
    submersible_test_motor = SubmersibleTestMotor()
    submersible_test_motor.export()





