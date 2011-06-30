"""
Creates an enclosure for the water channel controller electronics.
"""
from py2scad import *

INCH2MM = 25.4

# Inside dimensions
x,y,z = 24.0*INCH2MM, 14.0*INCH2MM, 4.0*INCH2MM
fn = 50

params = {
        'inner_dimensions'        : (x,y,z), 
        'wall_thickness'          : 6.0, 
        'lid_radius'              : 0.25*INCH2MM,  
        'top_x_overhang'          : 0.5*INCH2MM,
        'top_y_overhang'          : 0.5*INCH2MM,
        'bottom_x_overhang'       : 0.5*INCH2MM,
        'bottom_y_overhang'       : 0.5*INCH2MM, 
        'lid2front_tabs'          : (0.2,0.5,0.8),
        'lid2side_tabs'           : (0.25, 0.75),
        'side2side_tabs'          : (0.5,),
        'lid2front_tab_width'     : 1.0*INCH2MM,
        'lid2side_tab_width'      : 1.0*INCH2MM, 
        'side2side_tab_width'     : 0.5*INCH2MM,
        'tab_depth_adjust'        : 0.0,
        'standoff_diameter'       : 0.5*INCH2MM,
        'standoff_offset'         : 0.05*INCH2MM,
        'standoff_hole_diameter'  : 0.196*INCH2MM, 
        'filter_holder_thickness' : 6.0,
        'filter_location'         : (1.25*INCH2MM - 0.325*INCH2MM,0),  # Jo added a change here
        'cover_thickness'         : 3.0,
        'hole_list'               : [],
        }


enclosure = Basic_Enclosure(params)
enclosure.make()

part_assembly = enclosure.get_assembly(explode=(0,0,5),show_top=True,show_filter_holder=True,show_bottom=True)
part_projection = enclosure.get_projection()

prog_assembly = SCAD_Prog()
prog_assembly.fn = fn 
prog_assembly.add(part_assembly)
prog_assembly.write('controller_enclosure_assembly.scad')

prog_projection = SCAD_Prog()
prog_projection.fn = fn 
prog_projection.add(part_projection)
prog_projection.write('controller_enclosure_projection.scad')





