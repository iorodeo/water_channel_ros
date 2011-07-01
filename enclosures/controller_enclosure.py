"""
Creates an enclosure for the water channel controller electronics.
"""
import scipy
from py2scad import *

INCH2MM = 25.4

class Controller_Enclosure(Basic_Enclosure):

    def __init__(self, params):
        self.params = params
        self.pcb_hole_pos = []
        self.create_power_cable_hole()
        self.create_laser_sensor_hole()
        self.create_controller_holes()
        self.create_signal_cond_holes()
        #self.create_grid_holes()

    def make(self):
        super(Controller_Enclosure,self).make()

    def create_power_cable_hole(self):
        """
        Create mounting hole for power cable
        """
        x_dim, y_dim, z_dim = self.params['inner_dimensions']
        diam = self.params['power_cable_hole_diam']
        rel_pos = self.params['power_cable_rel_pos']
        x_pos = rel_pos*0.5*x_dim 
        hole = {
                'panel' : 'front',
                'type'  : 'round',
                'location' : (x_pos,0),
                'size'  :  diam,
                }
        self.params['hole_list'].append(hole)

    def create_laser_sensor_hole(self):
        """
        Create mounting hole for laser sensor
        """
        x_dim, y_dim, z_dim = self.params['inner_dimensions']
        diam = self.params['laser_sensor_hole_diam']
        rel_pos = self.params['laser_sensor_rel_pos']
        x_pos = rel_pos*0.5*x_dim 
        hole = {
                'panel' : 'back',
                'type'  : 'round',
                'location' : (x_pos,0),
                'size'  :  diam,
                }
        self.params['hole_list'].append(hole)


    def create_controller_holes(self):
        """
        Create holes for mounting controller pcb
        """
        x_dim, y_dim, z_dim = self.params['inner_dimensions']
        x_space, y_space = self.params['controller_holes_space']
        diam = self.params['controller_holes_diam']
        x_rel_pos, y_rel_pos = self.params['controller_rel_pos']
        self.controller_x = x_rel_pos*0.5*x_dim
        self.controller_y = y_rel_pos*0.5*y_dim
        hole_list = []
        for i in (-1,1):
            for j in (-1,1):
                x_pos = i*0.5*x_space + self.controller_x
                y_pos = j*0.5*y_space + self.controller_y
                hole = {
                        'panel' : 'bottom',
                        'type'  : 'round', 
                        'location' : (x_pos, y_pos),
                        'size' : diam,
                        }
                hole_list.append(hole)
                self.pcb_hole_pos.append((x_pos, y_pos))
        self.params['hole_list'].extend(hole_list)

    def create_signal_cond_holes(self):
        """
        Create holes for mounting signal conditioner
        """
        x_dim, y_dim, z_dim = self.params['inner_dimensions']
        x_space, y_space = self.params['signal_cond_holes_space']
        diam = self.params['signal_cond_holes_diam']
        x_rel_pos, y_rel_pos = self.params['signal_cond_rel_pos']
        self.signal_cond_x = x_rel_pos*0.5*x_dim
        self.signal_cond_y = y_rel_pos*0.5*y_dim
        hole_list = []
        for i in (-1,1):
            for j in (-1,1):
                x_pos = i*0.5*x_space + self.signal_cond_x
                y_pos = j*0.5*y_space + self.signal_cond_y
                hole = {
                        'panel' : 'bottom',
                        'type'  : 'round', 
                        'location' : (x_pos, y_pos),
                        'size' : diam,
                        }
                hole_list.append(hole)
                self.pcb_hole_pos.append((x_pos,y_pos))
        self.params['hole_list'].extend(hole_list)

    def create_grid_holes(self):
        """
        Create grid on holes on bottom of enclosure
        """
        # Get parameters
        x_dim, y_dim, z_dim = self.params['inner_dimensions']
        diam = self.params['grid_holes_diam']
        spacing = self.params['grid_holes_space']
        wall_gap = self.params['grid_holes_wall_gap']
        pcb_gap = self.params['grid_holes_pcb_gap']
        x_controller_space, y_controller_space = self.params['controller_holes_space']
        x_signal_cond_space, y_signal_cond_space = self.params['signal_cond_holes_space']

        ## Get pontential hole positions
        nx = scipy.floor((x_dim - 2*wall_gap)/spacing)
        ny = scipy.floor((y_dim - 2*wall_gap)/spacing)
        x_dist = (nx-1)*spacing
        y_dist = (ny-1)*spacing
        x_pos_array = scipy.linspace(-0.5*x_dist, 0.5*x_dist, nx)
        y_pos_array = scipy.linspace(-0.5*y_dist, 0.5*y_dist, ny)

        # Create holes
        hole_list = []
        for x_pos in x_pos_array:
            for y_pos in y_pos_array:
                near_pcb_hole= False
                # Test hole is too near pcb mount hole
                for x_pcb, y_pcb in self.pcb_hole_pos:
                    dist = get_dist((x_pcb,y_pcb),(x_pos,y_pos))
                    if dist < pcb_gap:
                        near_pcb_hole = True

                # Make holes if not inside pcb footprint
                if not near_pcb_hole: 
                    hole = {
                            'panel' : 'bottom', 
                            'type'  : 'round', 
                            'location' : (x_pos, y_pos),
                            'size' : diam,
                            }
                    hole_list.append(hole)
        self.params['hole_list'].extend(hole_list)

def get_dist(p0,p1):
    """
    Distance between two points
    """
    x0,y0 = p0
    x1,y1 = p1
    return scipy.sqrt((x0-x1)**2 + (y0-y1)**2)
        

#------------------------------------------------------------------------------
if __name__ == '__main__':

    # Inside dimensions
    x,y,z = 26.0*INCH2MM, 14.0*INCH2MM, 4.0*INCH2MM
    fn = 50
    
    params = {
            'inner_dimensions'         : (x,y,z), 
            'wall_thickness'           : 6.0, 
            'lid_radius'               : 0.25*INCH2MM,  
            'top_x_overhang'           : 0.5*INCH2MM,
            'top_y_overhang'           : 0.5*INCH2MM,
            'bottom_x_overhang'        : 0.5*INCH2MM,
            'bottom_y_overhang'        : 0.5*INCH2MM, 
            'lid2front_tabs'           : (0.2,0.4,0.6,0.8),
            'lid2side_tabs'            : (0.25,0.5,0.75),
            'side2side_tabs'           : (0.5,),
            'lid2front_tab_width'      : 1.0*INCH2MM,
            'lid2side_tab_width'       : 1.0*INCH2MM, 
            'side2side_tab_width'      : 0.5*INCH2MM,
            'tab_depth_adjust'         : 0.0,
            'standoff_diameter'        : 0.5*INCH2MM,
            'standoff_offset'          : 0.05*INCH2MM,
            'standoff_hole_diameter'   : 0.196*INCH2MM, 
            'filter_holder_thickness'  : 6.0,
            'filter_location'          : (1.25*INCH2MM - 0.325*INCH2MM,0),  # Jo added a change here
            'cover_thickness'          : 3.0,
            'hole_list'                : [],
            'controller_holes_space'   : (8.5*INCH2MM, 4.41*INCH2MM),
            'controller_holes_diam'    : 0.12*INCH2MM,
            'controller_rel_pos'       : (-0.5, -0.15), 
            'signal_cond_holes_space'  : (2.2788*INCH2MM,2.0*INCH2MM),
            'signal_cond_holes_diam'   : 0.12*INCH2MM,
            'signal_cond_rel_pos'      : (-0.5, 0.7),
            'grid_holes_diam'          : 0.25*INCH2MM,
            'grid_holes_space'         : 1.0*INCH2MM,
            'grid_holes_wall_gap'      : 0.5*INCH2MM,
            'grid_holes_pcb_gap'       : 0.5*INCH2MM, 
            'power_cable_hole_diam'    : 0.84*INCH2MM,
            'power_cable_rel_pos'      : 0.85,
            'laser_sensor_hole_diam'   : 0.84*INCH2MM,
            'laser_sensor_rel_pos'     : 0.6, 
            }
    
    
    enclosure = Controller_Enclosure(params)
    enclosure.make()
    
    part_assembly = enclosure.get_assembly(
            explode=(0,0,5),
            show_top=False,
            show_bottom=True)
    part_projection = enclosure.get_projection()
    
    prog_assembly = SCAD_Prog()
    prog_assembly.fn = fn 
    prog_assembly.add(part_assembly)
    prog_assembly.write('controller_enclosure_assembly.scad')
    
    prog_projection = SCAD_Prog()
    prog_projection.fn = fn 
    prog_projection.add(part_projection)
    prog_projection.write('controller_enclosure_projection.scad')





