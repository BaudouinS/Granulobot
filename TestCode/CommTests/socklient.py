""" Test Client Program for Socket Communications

    Test result: 10000 communications per second

"""

import socket
import sys
import time

# Put serveraddr in /run, /var/run or /tmp
serveraddr = '/tmp/gbotele'
bufsize = 5000

commands = ['something %d',
            'botdata %d',
            'else %d',
            'botdata %d',
            'exit %d']

for cnt in range(len(commands)):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(serveraddr)
    message = commands[cnt] % cnt
    print('Ready')
    s.sendall(message.encode())
    print('Sent')
    data=s.recv(bufsize)
    s.close()
    print("Got Data: %s" % data.decode())
    time.sleep(2.0)
    