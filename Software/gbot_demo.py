""" Granular Robots Demo program
    ============================
    
    Simple program to show how to read data and send commands through
    gbot_telemetry. This program needs gbot_telemetry to be running and
    have access to the configuration files.
    
    For more details on the options of gbutils.getelemetry and
    gbutils.sendcommand look at the GBot System Description document
    on the google drive.
    
    Usage: python gbot_demo.py configfile.txt
    
    Author: Marc Berthoud - mgb11@cornell.edu
    
"""

# Imports
import sys
from gbotlib import gbutils # Library to read / send gbot commands

# Read configuration
config = gbutils.setconfig(sys.argv)

# Get and print current telemetry
# Details on the getelemetry command are described in gbutils.py
teleraw = gbutils.getelemetry(config, 'raw')
print(teleraw)

# Send a poke command to all robots
# Details on sending commands to the robots are described in the
# help section of the command agent of gbut_termui.
gbutils.sendcommand(config, 'all p')