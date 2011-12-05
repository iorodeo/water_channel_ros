# Notes, may want to move these to the wc_params.xml file
DEFAULT_START_POSITION = 3.0
DEFAULT_AUTORUN_CHECK = False
DEFAULT_AUTORUN_DELAY = 5.0
MIN_AUTORUN_DELAY = 0
MAX_AUTORUN_DELAY = 999999 

STATUS_WINDOW_BACKGROUND_COLOR = (40,40,40)
STATUS_WINDOW_TEXT_COLOR = (255,140,0)

ALLOWED_STARTUP_MODES = [
        'captive trajectory',
        'position trajectory',
        'inertial sled',
        ]

# Timer update rates
IO_MODE_CHECK_TIMER_DT = 100   # ms
DIST_VELO_MSG_TIMER_DT = 100   # ms
PROGRESS_BAR_TIMER_DT = 100    # ms
STATUS_MESSAGE_TIMER_DT = 100  # ms
OUTSCAN_MONITOR_TIMER_DT = 100 # ms
