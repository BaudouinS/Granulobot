""" SimBot

    Simulates a granular robot by sending telemetry string and accepting commands.
    
    Special Commands: This robot has a few extra commands
    * exit: ends the simulated robot

"""

### setup
# Imports
import socket
import sys
import time
import select
from gbotlib import gbutils
# Variables
comm = '' # current command


### Initializations
# Open config file
config = gbutils.setconfig(sys.argv)

# Set up TCP listener for commands
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#sock.setblocking(False)
localaddr = (config['sockets']['telehost'], int(config['sockets']['cmdport']))
serveraddr = (config['sockets']['telehost'], int(config['sockets']['teleport']))
sock.bind(localaddr)
sock.listen(5)
print('Server Listening at %s:%d' % localaddr)
# Set up UDP telemetry communications
teleconn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Read messages to send
msgs = [ s.strip() for s in config['simbot']['teletext'].split('\n') if len(s) > 5]

### Command loop
msgi = 0 # next message to send
while not 'exit' in comm[:4].lower():
    ### Send the telemetry string
    teleconn.sendto(msgs[msgi].encode(), serveraddr)
    print('Sent %s' % msgs[msgi])
    # Increase message counter
    msgi += 1
    if not msgi < len(msgs): msgi = 0 
    ### Check for  incoming commands
    readlist, _wtl, _erl = select.select([sock], [], [],0.0)
    if readlist:
        # Receive command
        # Warning: Recv can block if sender delays send after connect.
        conn, cliaddr = sock.accept()
        comm = conn.recv(1024).decode()
        conn.close()
        print('Got Command %s' % comm)
    ### Pause
    time.sleep(1.0/float(config['simbot']['telefreq']))
    
""" To send data to it:
srv = ('192.168.1.15',6808)
msg = 'Hello'
s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(srv)
s.sendall(msg.encode())
s.close()
"""