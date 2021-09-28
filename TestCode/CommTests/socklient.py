""" Test Client Program for Socket Communications

    Test result: 10000 communications per second

"""

import socket
import sys

serveraddr = './testhost'
bufsize = 5000

for cnt in range(100000):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(serveraddr)
    message = 'Gimme Data %d' % cnt
    print('Ready')
    s.sendall(message.encode())
    print('Sent')
    data=s.recv(bufsize)
    s.close()
    print("Got Data: %s" % data.decode()[cnt//100:cnt//100+30])
    