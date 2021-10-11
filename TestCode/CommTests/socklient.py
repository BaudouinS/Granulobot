""" Test Client Program for Socket Communications

    Test result: 10000 communications per second with long text

"""

import socket
import sys
import time

# Put serveraddr in /run, /var/run or /tmp
#serveraddr = '/tmp/gbotele'
serveraddr = '/tmp/gbotcom'
bufsize = 1024

commands = ['something %d', 'botdata %d', # Commands for telemetry
            'else %d', 'botdata %d', 'exit %d']

commands = ['7 MOVE %d','3 Noone %d', '5 Stop %d','exit %d'] # Commands for commanding robots

for cnt in range(len(commands)):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(serveraddr)
    message = commands[cnt] % cnt
    print('Ready')
    s.sendall(message.encode())
    print('Sent')
    #data=s.recv(bufsize)
    s.close()
    #print("Got Data: %s" % data.decode())
    time.sleep(5.0)
    