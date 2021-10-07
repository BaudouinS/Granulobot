""" Test UDP Server Pogram for Socket Communicatdions

""" 

import socket
import sys
import os
from gbotlib import gbutils

config = gbutils.setconfig(sys.argv)
serveraddr = (config['sockets']['telehost'], int(config['sockets']['teleport']))

text = "Aeneadum genetrix, hominum divomque voluptas, alma Venus, caeli subter labentia signa quae mare navigerum, quae terras frugiferentis concelebras, per te quoniam genus omne animantum concipitur visitque exortum lumina solis: te, dea, te fugiunt venti, te nubila caeli adventumque tuum, tibi suavis daedala tellus summittit flores, tibi rident aequora ponti placatumque nitet diffuso lumine caelum. nam simul ac species patefactast verna diei et reserata viget genitabilis aura favoni, aeriae primum volucris te, diva, tuumque significant initum perculsae corda tua vi. inde ferae pecudes persultant pabula laeta et rapidos tranant amnis: ita capta lepore te sequitur cupide quo quamque inducere pergis. denique per maria ac montis fluviosque rapacis frondiferasque domos avium camposque virentis omnibus incutiens blandum per pectora amorem efficis ut cupide generatim saecla propagent. quae quoniam rerum naturam sola gubernas nec sine te quicquam dias in luminis oras exoritur neque fit laetum neque amabile quicquam, te sociam studeo scribendis versibus esse, quos ego de rerum natura pangere conor Memmiadae nostro, quem tu, dea, tempore in omni omnibus ornatum voluisti excellere rebus. quo magis aeternum da dictis, diva, leporem. effice ut interea fera moenera militiai per maria ac terras omnis sopita quiescant;"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(serveraddr)
print(len(text))
print('Server Listening at %s' % repr(serveraddr))

while True:
    data, clientaddr = sock.recvfrom(1024)
    print('connection from %s:%d' % clientaddr)
    print('received data "%s"' % data.decode())
    