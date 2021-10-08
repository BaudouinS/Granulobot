""" GBOT_READER

    Data reader for the Granular Robots project.
    This program receives data from the robots, puts all received
    messages in a file and makes the data available for query on
    an internal port.
"""

### Setup
# Imports
import sys
import os
import time
from datetime import datetime
import threading
import socket
import json
from gbotlib import gbutils

# Variables
botele = {} # dictionary with data for each robot
config = None
telefile = None
comm = ''

# Functions
def telerx():
    """ Telemetry Receiving Loop
    """
    global botele
    # Set up socket
    serveraddr = (config['sockets']['telehost'], int(config['sockets']['teleport']))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(serveraddr)
    print('Telemetry Server Listening')
    # Main loop
    while True:
        data, cliip = ['','']
        try:
            # Wait for new data package
            data, (cliip, cliport) = sock.recvfrom(1024)
            data = data.decode()
            # Get ID from data
            idpar = config['read']['idparam'].strip()
            #botid = [ s.strip()[len(idpar)+1:]
            #         for s in data.split()
            #         if idpar+'=' in s][0]
            botid = json.loads(data)[idpar]
            # Set values and add data to file
            data = data.strip() + ' IP=' + cliip
            botele[botid] = data
            dataline = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            dataline += ' %s: ' % botid
            dataline += data.strip()
            dataline += '\n'
            telefile.write(dataline)
            telefile.flush()
        except BaseException as e:
            print('Error receiving from %s message %s' % (cliip, data))
            raise e # in case there's a bug
        
def commloop():
    """ Commands and queries receiving loop
    """
    # Setup command message socket
    comsock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    comsock.bind(config['sockets']['telesock'])
    comsock.listen(5)
    print('Listening for Commands')
    while True:
        conn, client = comsock.accept()
        threading.Thread(target = commhandle, daemon = True,
                         args = (conn, client) ).start()
    
def commhandle(conn, client):
    """ Commands and queries handling function
    """
    global comm
    print('connection from %s' % client)
    data = conn.recv(1024)
    print('received command "%s"' % data.decode())
    comm = data.decode().strip()
    if 'botdata' in comm[:8].lower():
        botdata = [botele[dat] for dat in botele ]
        botdata = '\n'.join(botdata)
        conn.sendall(botdata.encode())
        conn.close()
    elif 'exit' in comm[:4].lower():
        print("That's All Folks!")
        conn.close()
    else:
        conn.close()
 

### Preparation
# Load configuration
config = gbutils.setconfig(sys.argv)
# Make sure command socket is not already in use
try:
    os.unlink(config['sockets']['telesock'])
except OSError:
    if os.path.exists(config['sockets']['telesock']):
        raise
# Open telemetry file
telefname = config['read']['telefile']
telefname = os.path.expandvars(telefname)
telefname = datetime.now().strftime(telefname)
telefile = open(telefname, 'wt')

# Setup  telemetry thread to get data from robots
telethr = threading.Thread(target = telerx, daemon = True)
telethr.start()
# Setup commands thread to get querries from other programs
commthr = threading.Thread(target = commloop, daemon = True)
commthr.start()

### Main Loop send data
while not 'exit' in comm[:4].lower():
    time.sleep(1.0) # because noone is in a hurry to check if we can exit
### close
telefile.close()
print('Last telementry entries:')
for botid in botele:
    print("bot %s - %s" % (botid, botele[botid]))