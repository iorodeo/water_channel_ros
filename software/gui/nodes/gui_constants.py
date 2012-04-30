import os

# Notes, may want to move these to the wc_params.xml file
DEFAULT_START_POSITION = 3.0
DEFAULT_AUTORUN_CHECK = False
DEFAULT_AUTORUN_DELAY = 5.0
MIN_AUTORUN_DELAY = 0
MAX_AUTORUN_DELAY = 999999 
AUTORUN_DELAY_LOOP_DT = 0.1
AUTORUN_DELAY_TIMER_MAX_COUNT = 100

STATUS_WINDOW_BACKGROUND_COLOR = (40,40,40)
STATUS_WINDOW_TEXT_COLOR = (255,140,0)

ALLOWED_STARTUP_MODES = [
        'captive trajectory',
        'position trajectory',
        'inertial sled',
        ]

# Timer update rates (ms)
IO_MODE_CHECK_TIMER_DT = 100   
DIST_VELO_MSG_TIMER_DT = 100   
PROGRESS_BAR_TIMER_DT = 100    
STATUS_MESSAGE_TIMER_DT = 100  
OUTSCAN_MONITOR_TIMER_DT = 100 
LOG_UPDATE_TIMER_DT = 500 

# Feedback positining parameters
FEEDBACK_POSITIONING_VELOCITY = 0.2
FEEDBACK_POSITIONING_ACCELERATION = 0.05

PLUGIN_DIRECTORY = os.path.join(os.environ['HOME'],'plugins')

# Default log file
DEFAULT_LOG_FILE = os.path.join(os.environ['HOME'],'default_log.hdf5')

