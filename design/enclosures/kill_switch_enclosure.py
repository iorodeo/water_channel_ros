"""
Creates an enclosure for the water channel controller kill switch.
"""
import scipy
from py2scad import *

class Kill_Switch_Enclosure(Basic_Enclosure):

    def __init__(self,params):
        self.params = params
        self.create_switch_hole()
        self.create_cord_grip_holes()
        self.create_mount_holes()

    def make(self):
        super(Kill_Switch_Enclosure,self).make()

    def create_switch_hole(self):
        """
        Create hole for rocker switch
        """
        x_dim, y_dim, z_dim = self.params['inner_dimensions']
        x_switch, y_switch = self.params['switch_cutout']
        hole = {
                'panel' : 'top',
                'type'  : 'square',
                'location' : (0.0,0.0),
                'size'  : (x_switch, y_switch),
                }
        self.params['hole_list'].append(hole)

    def create_cord_grip_holes(self):
        """
        Create holes for cord grips
        """
        x_dim, y_dim, z_dim = self.params['inner_dimensions']
        hole_offset = self.params['cord_grip_hole_offset']
        hole_diam = self.params['cord_grip_hole_diameter']
        hole_list = []
        for i in (-1,1):
            pos = i*hole_offset
            hole = {
                    'panel'    : 'left',
                    'type'     : 'round',
                    'location' : (pos,0),
                    'size'     : hole_diam,
                    }
            hole_list.append(hole)
        self.params['hole_list'].extend(hole_list)

    def create_mount_holes(self):
        """
        Create mount hole for enclosure
        """
        spacing_x, spacing_y = self.params['mount_hole_spacing']
        hole_diam = self.params['mount_hole_diameter']
        hole_list = []
        for i in (-1,1):
            for j in (-1,1):
                x = i*0.5*spacing_x
                y = j*0.5*spacing_y
                hole = {
                        'panel'    : 'bottom',
                        'type'     : 'round',
                        'location' : (x,y),
                        'size'     : hole_diam,
                        }
                hole_list.append(hole)
        self.params['hole_list'].extend(hole_list)




# -----------------------------------------------------------------------------
if __name__ == "__main__":

    # Inside dimensions
    x,y,z = 4.0*INCH2MM, 3.0*INCH2MM, 1.5*INCH2MM
    wall_thickness = (1.0/8.0)*INCH2MM
    fn = 50

    params = {
            'inner_dimensions'        : (x,y,z), 
            'wall_thickness'          : wall_thickness, 
            'lid_radius'              : 0.25*INCH2MM,  
            'top_x_overhang'          : 0.2*INCH2MM,
            'top_y_overhang'          : 0.2*INCH2MM,
            'bottom_x_overhang'       : 0.75*INCH2MM,
            'bottom_y_overhang'       : 0.2*INCH2MM, 
            'lid2front_tabs'          : (0.2,0.5,0.8),
            'lid2side_tabs'           : (0.25, 0.75),
            'side2side_tabs'          : (0.5,),
            'lid2front_tab_width'     : 0.4*INCH2MM,
            'lid2side_tab_width'      : 0.4*INCH2MM, 
            'side2side_tab_width'     : 0.4*INCH2MM,
            'standoff_diameter'       : 0.25*INCH2MM,
            'standoff_offset'         : 0.05*INCH2MM,
            'standoff_hole_diameter'  : 0.116*INCH2MM, 
            'switch_cutout'           : (0.76*INCH2MM, 0.51*INCH2MM),
            'cord_grip_hole_diameter' : 0.5*INCH2MM,
            'cord_grip_hole_offset'   : 0.5*INCH2MM,
            'mount_hole_diameter'     : 0.26*INCH2MM,
            'mount_hole_spacing'      : (5.0*INCH2MM, 2.0*INCH2MM),
            'hole_list'               : [],
            }

    enclosure = Kill_Switch_Enclosure(params)
    enclosure.make()

    part_assembly = enclosure.get_assembly(
            explode=(2,2,2),
            show_top=True,
            show_left=True,
            show_right=True,
            show_front=True,
            show_back=True,
            show_bottom=True,
            )
    part_projection = enclosure.get_projection()

    prog_assembly = SCAD_Prog()
    prog_assembly.fn = fn 
    prog_assembly.add(part_assembly)
    prog_assembly.write('kill_switch_enclosure_assembly.scad')

    prog_projection = SCAD_Prog()
    prog_projection.fn = fn 
    prog_projection.add(part_projection)
    prog_projection.write('kill_switch_enclosure_projection.scad')
