"""
Creates an enclosure for the water channel test motor electronics.
"""
import scipy
from py2scad import *

INCH2MM = 25.4

class Motor_Enclosure(Basic_Enclosure):
    
    def __init__(self, params):
        self.params = params
        self.create_power_supply_holes()
        self.create_cable_tie_holes()
        self.create_power_cable_hole()
        self.create_motor_power_hole()
        self.create_switch_hole()

    def make(self):
        super(Motor_Enclosure,self).make()
        self.make_power_supply()
        self.make_bnc_hole()

    def make_power_supply(self):
        """
        Create power supply object
        """
        x,y,z = self.params['power_supply_dimensions']
        terminal_x = self.params['power_supply_terminal_x']
        power_supply = Cube(size=(x,y,z))
        terminal = Cube(size=(terminal_x, 0.9*y, terminal_x))
        x_shift = -0.5*x - 0.5*terminal_x
        terminal = Translate(terminal, v=(x_shift, 0, 0))
        self.power_supply = Union([power_supply, terminal])

    def create_power_supply_holes(self):
        """
        Create mounting holes for power supply
        """
        x_dim, y_dim, z_dim = self.params['inner_dimensions']
        mount_space_x, mount_space_y = self.params['power_supply_mount_space']
        mount_gap = self.params['power_supply_mount_gap']
        mount_diam = self.params['power_supply_mount_diam']

        self.power_supply_pos_x = - 0.5*x_dim + 0.5*mount_space_x + mount_gap
        self.power_supply_pos_y = 0 
        hole_list = []
        for i in (-1,1):
            for j in (-1,1):
                x_pos = i*0.5*mount_space_x + self.power_supply_pos_x 
                y_pos = j*0.5*mount_space_y + self.power_supply_pos_y
                hole = {
                        'panel' : 'bottom', 
                        'type'  : 'round', 
                        'location' : (x_pos, y_pos),
                        'size'  : mount_diam,
                        }
                hole_list.append(hole)

        self.params['hole_list'].extend(hole_list)

    def create_cable_tie_holes(self):
        """
        Create holes for cables on bottom on enclosure
        """
        x_dim, y_dim, z_dim = self.params['inner_dimensions']
        x_supply, y_supply, z_supply = self.params['power_supply_dimensions']
        x_gap, y_gap = self.params['cable_tie_hole_gap']
        diam = self.params['cable_tie_hole_diam']
        nx,ny = self.params['cable_tie_num']

        hole_list = []
        x_pos = scipy.linspace(-0.5*x_dim + x_gap, 0.5*x_dim - x_gap, nx)
        y_pos = scipy.linspace(-0.5*y_dim + y_gap, 0.5*y_dim - y_gap, ny)
        for x in x_pos:
            for y in y_pos:
                if y < 0.5*y_supply and y > -0.5*y_supply and x < self.power_supply_pos_x + 0.5*x_supply:
                    continue
                hole = {
                        'panel' : 'bottom',
                        'type'  : 'round', 
                        'location' : (x,y), 
                        'size' : diam,
                        }
                hole_list.append(hole)
        self.params['hole_list'].extend(hole_list)

    def create_power_cable_hole(self):
        diam = self.params['power_cable_hole_diam']
        hole = {
                'panel' : 'left',
                'type'  : 'round',
                'location' : (0,0),
                'size'  :  diam,
                }
        self.params['hole_list'].append(hole)

    def create_motor_power_hole(self):
        diam = self.params['motor_power_hole_diam']
        hole = {
                'panel' : 'right',
                'type'  : 'round',
                'location' : (0,0),
                'size'  :  diam,
                }
        self.params['hole_list'].append(hole)

    def create_switch_hole(self):
        x_dim, y_dim, z_dim = self.params['inner_dimensions']
        x_switch, y_switch = self.params['switch_cutout']
        rel_pos_x = self.params['switch_rel_pos_x']
        x_pos = rel_pos_x*x_dim
        y_pos = 0.0

        hole = {
                'panel' : 'back',
                'type'  : 'square',
                'location' : (x_pos,y_pos),
                'size'  : (x_switch, y_switch),
                }

        self.params['hole_list'].append(hole)

    def make_bnc_hole(self):
        """
        This is a special hole, so we can't use the normal hole making
        tools.
        """
        x_dim, y_dim, z_dim = self.params['inner_dimensions']
        wall_thickness = self.params['wall_thickness']
        diam = self.params['bnc_diam']
        rel_pos_x = self.params['bnc_rel_pos_x']
        radius = 0.5*diam
        cutoff = self.params['bnc_cutoff']
        cut_h = 2*self.params['wall_thickness']
        cut_cyl = Cylinder(h=cut_h,r1=radius, r2=radius) 
        cut_block = Cube(size=(2*diam, 2*diam, 2*cut_h))
        x_shift = 0.5*diam + diam - (0.5*diam - cutoff)
        cut_block = Translate(cut_block, (x_shift, 0, 0))
        cut_cyl = Difference([cut_cyl, cut_block]) 
        cut_cyl = Rotate(cut_cyl, a=90, v=(0,0,1))
        #cut_cyl = Rotate(cut_cyl, a=-90, v=(1,0,0))
        cut_cyl = Translate(cut_cyl, v=(rel_pos_x*x_dim,0,0)) #0.5*y_dim+0.5*wall_thickness,0))
        self.back = Difference([self.back, cut_cyl])

    def get_assembly(self, **kwargs):
        assembly_options = {
                'explode'            : (0,0,0),
                'show_power_supply'  : True, 
                }
        assembly_options.update(kwargs)
        explode = assembly_options['explode']
        explode_x, explode_y, explode_z = explode

        x_dim, y_dim, z_dim = self.params['inner_dimensions']

        parts_list = super(Motor_Enclosure, self).get_assembly(**kwargs)

        # Translate power supply into position
        x_supply, y_supply, z_supply = self.params['power_supply_dimensions']
        power_supply = self.power_supply
        power_supply = Color(power_supply, rgba=self.params['power_supply_color'])
        x_shift = self.power_supply_pos_x
        y_shift = self.power_supply_pos_y
        z_shift = -0.5*z_dim + 0.5*z_supply
        power_supply = Translate(power_supply, v=(self.power_supply_pos_x, self.power_supply_pos_y, z_shift))

        if assembly_options['show_power_supply'] == True:
            parts_list.append(power_supply)

        return parts_list



# -----------------------------------------------------------------------------
if __name__ == "__main__":

    # Inside dimensions
    x,y,z = 15.0*INCH2MM, 10.0*INCH2MM, 2.5*INCH2MM
    fn = 50
    
    params = {
            'inner_dimensions'            : (x,y,z), 
            'wall_thickness'              : 6.0, 
            'lid_radius'                  : 0.25*INCH2MM,  
            'top_x_overhang'              : 0.5*INCH2MM,
            'top_y_overhang'              : 0.5*INCH2MM,
            'bottom_x_overhang'           : 0.5*INCH2MM,
            'bottom_y_overhang'           : 0.5*INCH2MM, 
            'lid2front_tabs'              : (0.2,0.5,0.8),
            'lid2side_tabs'               : (0.25, 0.75),
            'side2side_tabs'              : (0.5,),
            'lid2front_tab_width'         : 1.0*INCH2MM,
            'lid2side_tab_width'          : 1.0*INCH2MM, 
            'side2side_tab_width'         : 0.5*INCH2MM,
            'tab_depth_adjust'            : 0.0,
            'standoff_diameter'           : 0.5*INCH2MM,
            'standoff_offset'             : 0.05*INCH2MM,
            'standoff_hole_diameter'      : 0.196*INCH2MM, 
            'filter_holder_thickness'     : 6.0,
            'filter_location'             : (1.25*INCH2MM - 0.325*INCH2MM,0),  # Jo added a change here
            'cover_thickness'             : 3.0,
            'hole_list'                   : [],
            'power_supply_dimensions'     : (215, 115, 50),
            'power_supply_terminal_x'     : 13.0,
            'power_supply_mount_space'    : (150.0, 50.0), 
            'power_supply_mount_gap'      : 90.0, 
            'power_supply_mount_diam'     : 4.5,
            'power_supply_color'          : (0,0.5,1,1), 
            'cable_tie_hole_diam'         : 0.25*INCH2MM,
            'cable_tie_hole_gap'          : (1.0*INCH2MM, 0.5*INCH2MM),
            'cable_tie_num'               : (20, 12),
            'power_cable_hole_diam'       : 0.84*INCH2MM,
            'motor_power_hole_diam'       : 0.84*INCH2MM,
            'bnc_diam'                    : 9.91,
            'bnc_cutoff'                  : 3.89,
            'bnc_rel_pos_x'               : 0.35,
            'switch_cutout'               : (0.76*INCH2MM, 0.51*INCH2MM),
            'switch_rel_pos_x'            : 0.2,
            }
    
    
    enclosure = Motor_Enclosure(params)
    enclosure.make()
    
    part_assembly = enclosure.get_assembly(
            explode=(0,0,0),
            show_top=True,
            show_left=True,
            show_right=True,
            show_front=True,
            show_back=True,
            show_bottom=True,
            show_power_supply=True,
            )


    part_projection = enclosure.get_projection()
    
    prog_assembly = SCAD_Prog()
    prog_assembly.fn = fn 
    prog_assembly.add(part_assembly)
    prog_assembly.write('motor_enclosure_assembly.scad')
    
    prog_projection = SCAD_Prog()
    prog_projection.fn = fn 
    prog_projection.add(part_projection)
    prog_projection.write('motor_enclosure_projection.scad')





