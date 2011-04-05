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
import cad.csg_objects as csg
import cad.finite_solid_objects as fso
import math
import copy


SUBMERSIBLE_MOUNT_PARAMETERS = {
    'x': 3.0,
    'y': 2.125,
    'z': 2.0,
    'profile': 'submersible_mount/submersible_mount.dxf',
    'color': [1.0,1.0,0.0,1.0],
    'hole_x': (-1.125,1.125),
    'hole_z': (-0.5,0.5),
    'hole_r': 0.129,
    'hole_l': 1,
    }

def get_parameters():
    return copy.deepcopy(SUBMERSIBLE_MOUNT_PARAMETERS)

class SubmersibleMount(csg.Difference):
    def __init__(self):
        super(SubmersibleMount, self).__init__()
        self.parameters = SUBMERSIBLE_MOUNT_PARAMETERS
        self.__make_submersible_mount()
        self.__make_holes()
        # self.rotate(angle=math.pi/2,axis=[1,0,0])
        self.rotate(angle=-math.pi/2,axis=[0,0,1])
        self.set_color([1,1,0],recursive=True)

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __make_submersible_mount(self):
        profile = self.parameters['profile']
        l = self.parameters['z']
        submersible_mount = fso.Extrusion(profile=profile,l=l)
        self.add_obj(submersible_mount)

    def __make_holes(self):
        hole_r = self.parameters['hole_r']
        hole_l = self.parameters['hole_l']
        hole_x = self.parameters['hole_x']
        hole_z = self.parameters['hole_z']
        hole_list = []
        base_hole = fso.Cylinder(r=hole_r,l=hole_l)
        base_x_hole = base_hole.copy()
        base_x_hole.rotate(angle=-math.pi/2,axis=[1,0,0])
        for z in hole_z:
            for x in hole_x:
                hole = base_x_hole.copy()
                hole.translate([x,0,z])
                hole_list.append(hole)
        self.add_obj(hole_list)


if __name__ == "__main__":
    import cad_library.origin as origin
    o = origin.Origin()
    submersible_mount = SubmersibleMount()
    # submersible_mount = submersible_mount | o
    submersible_mount.export()





