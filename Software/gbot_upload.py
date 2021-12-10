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
import os, sys, logging
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
log = logging.getLogger('Upload')
### Get database of robots from latest datalog
# Find latest logfile (go back by hours until found or Nhours > 24
backsec = 0
telefname = ''
for backsec in range(0,86400,900):
    telefname = config['telemetry']['telefile']
    telefname = os.path.expandvars(telefname)
    telefname = (datetime.now()-timedelta(seconds=backsec)).strftime(telefname)
    log.debug('Looking for %s' % telefname)
    if os.path.exists(telefname):
        break
    telefname = ''
if not telefname:
    msg = 'Unable to find Telemetry file - exiting'
    log.error(msg)
    raise(RuntimeError(msg))
log.info('Found telemetry file %s' % telefname)
# Get robot entries from telemetry file
telines = [l.strip() for l in open(telefname) if len(l) > 5]
botele = {}
for l in telines[-10:]:
    # Get json
    rjson = json.loads(l[l.index('{'):])
    botele[rjson[config['telemetry']['idparam']]]=rjson
for r in botele:
    log.debug('Bot %s: %s' % (repr(r), repr(botele[r])))
### Upload to the robots
for r in botele:
    # Get file (run expandvars once here, once later)
    binfile = os.path.expandvars(config['upload'][str(r)])
    cmd = config['upload']['cmd'] % (botele[r]['ip'], binfile)
    cmd = os.path.expandvars(cmd)
    log.info('Running %s' % cmd)
    os.system(cmd)
