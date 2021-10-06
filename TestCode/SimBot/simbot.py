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

### Command loop
while not 'exit' in comm[:4].lower():
    ### Send the telemetry string
    message = config['simbot']['teletext']
    teleconn.sendto(message.encode(), serveraddr)
    print('Sent %s' % message)
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