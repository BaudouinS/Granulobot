""" Test Server Pogram for Socket Communicatdions

""" 

import socket
import sys
import os

serveraddr = '/tmp/testsock'

text = "Aeneadum genetrix, hominum divomque voluptas, alma Venus, caeli subter labentia signa quae mare navigerum, quae terras frugiferentis concelebras, per te quoniam genus omne animantum concipitur visitque exortum lumina solis: te, dea, te fugiunt venti, te nubila caeli adventumque tuum, tibi suavis daedala tellus summittit flores, tibi rident aequora ponti placatumque nitet diffuso lumine caelum. nam simul ac species patefactast verna diei et reserata viget genitabilis aura favoni, aeriae primum volucris te, diva, tuumque significant initum perculsae corda tua vi. inde ferae pecudes persultant pabula laeta et rapidos tranant amnis: ita capta lepore te sequitur cupide quo quamque inducere pergis. denique per maria ac montis fluviosque rapacis frondiferasque domos avium camposque virentis omnibus incutiens blandum per pectora amorem efficis ut cupide generatim saecla propagent. quae quoniam rerum naturam sola gubernas nec sine te quicquam dias in luminis oras exoritur neque fit laetum neque amabile quicquam, te sociam studeo scribendis versibus esse, quos ego de rerum natura pangere conor Memmiadae nostro, quem tu, dea, tempore in omni omnibus ornatum voluisti excellere rebus. quo magis aeternum da dictis, diva, leporem. effice ut interea fera moenera militiai per maria ac terras omnis sopita quiescant;"

# Check that socket is unused
try:
    os.unlink(serveraddr)
except OSError:
    if os.path.exists(serveraddr):
        raise

sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.bind(serveraddr)
sock.listen(5)
print(len(text))
print('Server Listening')

while True:
    conn, client = sock.accept()
    print('connection from %s' % client)
    data = conn.recv(1024)
    print('received data "%s"' % data.decode())
    conn.sendall(text.encode())
    conn.close()
    print('Closed')
    