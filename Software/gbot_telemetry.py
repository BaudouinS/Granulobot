""" GBOT_TELEMETRY

    Data reader for the Granular Robots project.
    This program receives data from the robots, puts all received
    messages in a file and makes the data available for query on
    an internal port.
    
    Usage:
        python gbot_telemetry.py configfile.txt
        
    See google doc for more information on using gbot_telemetry.
    
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
import logging
import logging.handlers
import socketserver
import pickle
import re
from gbotlib import gbutils

# Variables
botele = {} # dictionary with data for each robot
botime = {} # dictionary with last timestamp for each robot
config = None
telefile = None
exitcmd = False # Boolean indicating if exit command has been set

# Functions
def teleloop():
    """ Telemetry Receiving Loop: Receives telemetry
        from the robots
    """
    global botele
    # Set up socket
    serveraddr = (config['sockets']['telehost'], int(config['sockets']['teleport']))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(serveraddr)
    log.info('Telemetry Server Listening')
    # Main loop
    while True:
        data, cliip = ['','']
        try:
            # Wait for new data package
            data, (cliip, cliport) = sock.recvfrom(1024)
            data = data.decode()
            # Get ID from data
            idpar = config['telemetry']['idparam'].strip()
            #botid = [ s.strip()[len(idpar)+1:]
            #         for s in data.split()
            #         if idpar+'=' in s][0]
            datajs = json.loads(data)
            botid = str(datajs[idpar])
            # Add IP Address
            #data = data.strip() + ' ip=' + cliip
            datajs["ip"]=cliip
            # Add time stamp
            datajs["time"]=datetime.now().timestamp()
            # Set values and add data to file
            data = json.dumps(datajs)
            botele[botid] = data
            dataline = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            dataline += ' %s: ' % botid
            dataline += data.strip()
            dataline += '\n'
            telefile.write(dataline)
            telefile.flush()
        except BaseException as e:
            log.warn('Error receiving telemetry from %s, message %s' % (cliip, data))
            raise e # in case there's a bug
        
def queryloop():
    """ Queries receiving loop: To answer queries about robot
        telemetry.
    """
    # Setup command message socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('127.0.0.1',int(config['sockets']['telesock'])))
    sock.listen(5)
    log.info('Listening for Querries')
    while True:
        conn, client = sock.accept()
        threading.Thread(target = queryhandle, daemon = True,
                         args = (conn, client) ).start()
    
def queryhandle(conn, client):
    """ Queries handling function
    """
    global exitcmd
    #print('query: connection from %s' % client)
    data = conn.recv(1024)
    #print('query: received command "%s"' % data.decode())
    comm = data.decode().strip()
    if 'botdata' in comm[:8].lower():
        botdata = [botele[dat] for dat in botele ]
        botdata = '\n'.join(botdata)
        conn.sendall(botdata.encode())
        conn.close()
    elif 'exit' in comm[:4].lower():
        print("That's All Folks!")
        exitcmd = True
        conn.close()
    else:
        conn.close()
 
def cmdloop():
    """ Command receiving loop: To receive commands to forward
        to the robots.
    """
    # Setup command message socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('127.0.0.1',int(config['sockets']['comsock'])))
    sock.listen(5)
    log.info('Listening for Commands')
    while True:
        conn, client = sock.accept()
        threading.Thread(target = cmdhandle, daemon = True,
                         args = (conn, client) ).start()

def cmdhandle(conn, client):
    """ Commands handling function
    """
    global exitcmd
    # Receive the command
    #print('comm: connection from %s' % client)
    data = conn.recv(1024)
    #print('comm: received command "%s"' % data.decode())
    log.debug('comm: received command "%s"' % data.decode())
    comm = data.decode().strip()
    # Close the connection
    conn.close()
    # Handle the command
    if 'exit' in comm[:4].lower():
        # Exit command -> Exit
        print("That's All Folks!")
        exitcmd = True
    else: # It's a command for a bot:
        # Get first word, ID of bots to contact
        idlen = comm.find(' ')
        botids = comm[:idlen].split('/')
        # Loop over botids
        for botid in botids:
            # Check if botid is valid
            if botid in botele:
                # Get a socket and address
                commsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                botel = json.loads(botele[botid])
                botaddr = (botel['ip'], int(config['sockets']['cmdport']))
                commsock.sendto(comm[idlen:].strip().encode(), botaddr)
            elif botid in 'all':
                # Send message to all robots
                commsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                for id in botele:
                    botel = json.loads(botele[id])
                    botaddr = (botel['ip'], int(config['sockets']['cmdport']))
                    commsock.sendto(comm[idlen:].strip().encode(), botaddr)
            elif re.search(r'[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+',botid):
                # Send message to IP address
                if '255' in botid[-3:]:
                    # Set socket for broadcast NEEDS TO BE TESTED
                    commsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    # The following lines could be needed but seem not to be
                    #                         socket.IPPROTO_UDP)
                    #commsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    #commsock.bind(('',0))
                    commsock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                else:
                    # Set for single IP address
                    commsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                botaddr = (botid, int(config['sockets']['cmdport']))
                commsock.sendto(comm[idlen:].strip().encode(), botaddr)
            else:
                log.warning('Invalid robot ID in <%s>' % comm)

def logloop():
    """ Logging Receiving Loop: Reveives log messages from
        internal programs
    """
    # Set up socket
    serveraddr = ('127.0.0.1', int(config['sockets']['logsock']))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(serveraddr)
    log.info('Logging Server Listening')
    # Main loop
    while True:
        data, cliip = [b'','']
        try:
            # Wait for new data package
            data, (cliip, cliport) = sock.recvfrom(4096)
            while False: #True:
                packet, (cliip, cliport) = sock.recvfrom(4096)
                print(repr(packet))
                if not packet: break
                data += packet
            logobj = pickle.loads(data[4:])
            logrec = logging.makeLogRecord(logobj)
            logging.getLogger(logrec.name).handle(logrec)
        except BaseException as e:
            log.warn('Error receiving log from %s, message %s' % (cliip, data))

### Preparation
# Load configuration
config = gbutils.setconfig(sys.argv)
# Set up logging
logging.basicConfig(level = 'DEBUG')
log = logging.getLogger('Telemetry')
logfname = config['logging']['logfile']
logfname = os.path.expandvars(logfname)
logfname = datetime.now().strftime(logfname)
logformat = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
fhand = logging.FileHandler(logfname)
fhand.setFormatter(logging.Formatter(logformat))
logging.getLogger().addHandler(fhand)
log.debug('Setup logging, logfile = %s' % logfname)
log.debug('Loaded Configuration = %s' % repr(config))
# Open telemetry file
telefname = config['telemetry']['telefile']
telefname = os.path.expandvars(telefname)
telefname = datetime.now().strftime(telefname)
telefile = open(telefname, 'wt')
# Setup  telemetry thread to get data from robots
telethr = threading.Thread(target = teleloop, daemon = True)
telethr.start()
# Setup query thread to get queries from other programs
querythr = threading.Thread(target = queryloop, daemon = True)
querythr.start()
# Setup command thread to send commands to the robots
cmdthr = threading.Thread(target = cmdloop, daemon = True)
cmdthr.start() 
# Setup logging loop to receive log messages
logthr = threading.Thread(target = logloop, daemon = True)
logthr.start()

### Main Loop send data
while not exitcmd:
    # because no one is in a hurry to check if we can exit
    time.sleep(1.0) 
### close
log.info('Last telementry entries:')
for botid in botele:
    log.info("bot %s - %s" % (botid, botele[botid]))
telefile.close()
