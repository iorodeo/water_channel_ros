from __future__ import division
import roslib
roslib.load_manifest('water_channel_mechanics')
import rospy
import copy

import cad.finite_solid_objects as fso
import cad.csg_objects as csg
import cad.export.bom as bom


# RAB air bearing parameters
# Data sheet parameter
RAB4_PARAMETERS = {                            # --------------------
        'slide_width'          : 4.0,      # A
        'slide_height'         : 1.5,      # B
        'carriage_length'      : 5.0,      # C
        'carriage_width'       : 5.5,      # D,E
        'carriage_height'      : 3.0,      # F
        'slide_screw_inset'    : 0.313,    # G
        'slide_screw_dW'       : 3.0,      # H
        'carriage_screw_dW'    : 4.75,     # J
        'carriage_screw_dL'    : 3.25,     # K
        'slide_base_length'    : 6.25,     # L
        'slide_screw_size'     : 0.25,     # M
        'carriage_screw_size'  : 0.25,     # N
        'slide_tolerance'      : 0.005,
        'name'                 : 'RAB4',
        'description'          : 'Non-motorized air bearing slide',
        'vendor'               : 'Nelson Air Corp.',
        'part number'          : 'RAB4',
        'cost'                 : 0.00,
        }

# Data sheet parameter
RAB6_PARAMETERS = {                            # --------------------
        'slide_width'          : 6.0,      # A
        'slide_height'         : 1.75,     # B
        'carriage_length'      : 7.0,      # C
        'carriage_width'       : 7.5,      # D,E
        'carriage_height'      : 3.25,     # F
        'slide_screw_inset'    : 0.313,    # G
        'slide_screw_dW'       : 5.0,      # H
        'carriage_screw_dW'    : 6.75,     # J
        'carriage_screw_dL'    : 5.00,     # K
        'slide_base_length'    : 8.25,     # L
        'slide_screw_size'     : 0.25,     # M
        'carriage_screw_size'  : 0.25,     # N
        'slide_tolerance'      : 0.005,
        'name'                 : 'RAB6',
        'description'          : 'Non-motorized air bearing slide',
        'vendor'               : 'Nelson Air Corp.',
        'part number'          : 'RAB6',
        'cost'                 : 4095.00,
        }

# Data sheet parameter
RAB10_PARAMETERS = {                           # --------------------
        'slide_width'          : 10.0,     # A
        'slide_height'         : 3.0,      # B
        'carriage_length'      : 12.0,     # C
        'carriage_width'       : 12.0,     # D,E
        'carriage_height'      : 5.5,      # F
        'slide_screw_inset'    : 0.313,    # G
        'slide_screw_dW'       : 9.0,      # H
        'carriage_screw_dW'    : 11.0,     # J
        'carriage_screw_dL'    : 9.50,     # K
        'slide_base_length'    : 13.25,    # L
        'slide_screw_size'     : 0.25,     # M
        'carriage_screw_size'  : 0.25,     # N
        'slide_tolerance'      : 0.005,
        'name'                 : 'RAB10',
        'description'          : 'Non-motorized air bearing slide',
        'vendor'               : 'Nelson Air Corp.',
        'part number'          : 'RAB10',
        'cost'                 : 0.00,
        }

BEARING_PARAMETERS = {
        'RAB4'  : RAB4_PARAMETERS,
        'RAB6'  : RAB6_PARAMETERS,
        'RAB10' : RAB10_PARAMETERS,
        }

class RAB(csg.Union):
    """
    Creates a model of the RAB air bearings.
    """

    def __init__(self,bearing_type,slide_travel,slide_color=[0.5,0.5,0.5],carriage_color=[0.5,0.5,0.5]):
        super(RAB, self).__init__()
        self.bearing_type = bearing_type
        self.parameters = BEARING_PARAMETERS[bearing_type]
        self.parameters['bearing_slide_travel'] = slide_travel
        self.slide_color = slide_color
        self.carriage_color = carriage_color
        self.__make_slide()
        self.__make_carriage()
        self.__make_slide_travel()
        self.__set_bom()
        self.set_obj_list([self.slide,self.carriage,self.slide_travel])

    def get_parameters(self):
        return copy.deepcopy(self.parameters)

    def __set_bom(self):
        BOM = bom.BOMObject()
        BOM.set_parameter('name',self.parameters['name'])
        BOM.set_parameter('description',self.parameters['description'])
        BOM.set_parameter('dimensions',('slide travel: ' + str(self.parameters['bearing_slide_travel'])))
        BOM.set_parameter('vendor',self.parameters['vendor'])
        BOM.set_parameter('part number',self.parameters['part number'])
        BOM.set_parameter('cost',self.parameters['cost'])
        self.set_object_parameter('bom',BOM)

    def set_slide_travel(self,val):
        self.parameters['bearing_slide_travel'] = val
        self.__make_slide()
        self.__make_carriage()
        self.__make_slide_travel()
        self.set_obj_list([self.slide,self.carriage,self.slide_travel])

    def __make_slide(self):
        """
        Creates the slide component of the RAB air bearing.
        """
        # Create base rectangle for slide
        length = self.parameters['slide_base_length'] + self.parameters['bearing_slide_travel']
        width = self.parameters['slide_width']
        height = self.parameters['slide_height']
        slide = fso.Box(x=length,y=width,z=height)
        # Create the mounting holes
        radius = 0.5*self.parameters['slide_screw_size']
        base_hole = fso.Cylinder(r=radius, l=2*height)
        hole_list = []
        for i in (-1,1):
            for j in (-1,1):
                xpos = i*(0.5*length - self.parameters['slide_screw_inset'])
                ypos = j*(0.5*self.parameters['slide_screw_dW'])
                hole = base_hole.copy()
                hole.translate([xpos,ypos,0])
                hole_list.append(hole)
        # Remove hole material
        slide -= hole_list
        slide.set_color(self.slide_color,recursive=True)
        self.slide = slide

    def __make_carriage(self):
        """
        Creates the carriage component of the RAB air bearing.
        """
        # Create base rectangle
        length = self.parameters['carriage_length']
        width = self.parameters['carriage_width']
        height = self.parameters['carriage_height']
        carriage = fso.Box(x=length, y=width, z=height)

        # Subtract slide from carraige
        slide_width = self.parameters['slide_width'] + 2*self.parameters['slide_tolerance']
        slide_height  = self.parameters['slide_height'] + 2*self.parameters['slide_tolerance']
        slide_negative = fso.Box(x=2*length, y=slide_width, z=slide_height)
        carriage = carriage - slide_negative

        # Create mounting holes
        radius = 0.5*self.parameters['carriage_screw_size']
        base_hole = fso.Cylinder(r=radius,l=2*height)
        hole_list = []
        for i in (-1,1):
            for j in (-1,1):
                xpos = i*0.5*self.parameters['carriage_screw_dL']
                ypos = j*0.5*self.parameters['carriage_screw_dW']
                hole = base_hole.copy()
                hole.translate([xpos,ypos,0])
                hole_list.append(hole)
        # Remove hole material
        # print hole_list
        carriage -= hole_list
        carriage.set_color(self.carriage_color,recursive=True)
        self.carriage = carriage

    def __make_slide_travel(self,color=[0.25,0.25,0.25]):
        """
        Make a colored region showing the slides travel.
        """
        length = self.parameters['carriage_length'] + self.parameters['bearing_slide_travel']
        width = self.parameters['slide_width'] + self.parameters['slide_tolerance']
        height = self.parameters['slide_height'] +  self.parameters['slide_tolerance']
        slide_travel = fso.Box(x=length,y=width,z=height)
        slide_travel.set_color(color,recursive=True)
        self.slide_travel = slide_travel


# ---------------------------------------------------------------------
if __name__ == '__main__':

    bearing_type = 'RAB6'
    slide_travel = 4

    bearing = RAB(bearing_type, slide_travel)
    # obj_list = bearing.get_obj_list()
    # for obj in obj_list:
    #     print obj.get_obj_list()
    # print bearing
    bearing.export()
