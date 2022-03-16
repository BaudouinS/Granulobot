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
    
    It is possible to specify which robots to upload to by listing
    the robot ids of the various robots.  If no IDs are specified
    the firmware is loaded to all robots.
    
    Usage:
        python gbot_uplaod.py configfile.txt id1 id2 id3

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
### Get robot entries from telemetry file
# Read all lines
telines = [l.strip() for l in open(telefname) if len(l) > 5]
# Add json of each robot into botele dictionary
# (multiple entries of same robot are overwritten to keep latest)
botele = {}
for l in telines:
    # Get json
    rjson = json.loads(l[l.index('{'):])
    botele[rjson[config['telemetry']['idparam']]]=rjson
for r in botele:
    log.debug('Bot %s: %s' % (repr(r), repr(botele[r])))
### Get select robot names from argv
botelesel = {}
for r in botele:
    if repr(r) in sys.argv:
        botelesel[r] = botele[r]
if len(botelesel):
    selbots = ' '.join([repr(r) for r in botelesel])
    log.debug(f'Selected {len(botelesel)} robots based on command arguments: ' +
              selbots)
    botele = botelesel
else:
    log.debug('No robots in command arguments, uploading to all')
### Upload to the robots
for r in botele:
    # Get file (run expandvars once here, once later)
    binfile = os.path.expandvars(config['upload'][str(r)])
    cmd = config['upload']['cmd'] % (botele[r]['ip'], binfile)
    cmd = os.path.expandvars(cmd)
    log.info('Running %s' % cmd)
    os.system(cmd)
