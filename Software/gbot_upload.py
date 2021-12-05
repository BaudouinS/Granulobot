""" GBOT_UPLOAD

    Program to upload firmware (usually .bin files) to the robots
    for the Granular Robots project. This program uses the command
    line OTA uploader at
        https://steve.fi/hardware/ota-upload
    
    gbot_upload uses the configuration file(s) as other gbot projects.
    The information which firmware file to load to each robot is
    detailed in the [upload] section of the config files. The program
    also uses information from other sections of the config file.
    Delta config files can be used.
    
    The program then uses the most recent datalog file to get the IP
    addresses for the robots.
    
    Usage:
        python gbot_uplaod.py configfile.txt

"""

### Setup
# Imports
import os
import json
from datetime import datetime, timedelta
from gbotlib import gbutils

### Preparation
# Load configuration
config = gbutils.setconfig(sys.argv)
# Set up logging
logging.basicConfig(level = 'DEBUG')
# Set up logging handler which will send logs to gbot_telemetry if it's running
gbutils.setuplogging(config)
log = logging.getlogger('Upload')
### Get database of robots from latest datalog
# Find latest logfile (go back by hours until found or Nhours > 24
backsec = 0
telfname = ''
for backsec in range(0,86400,300):
    telfname = config['telemetry']['telefile']
    telefname = os.path.expandvars(telefname)
    telefname = (datetime.now()+timedelta(seconds=backsec)).strftime(telefname)
    log.debug('Looking for %s' % telfname)
    if os.path.exists(telfname):
        break
    telfname = ''
if not telfname:
    log.error('Unable to find Telemetry file - exiting')
    return 1
# Get robot entries from all telemetry file
telines = [l.strip() for l in open(telfname) if len(l) > 5]
botele = {}
for l in telines:
    
### Upload to the robots

