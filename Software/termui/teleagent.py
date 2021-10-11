""" Interface Agent Control Program - Telemetry Agent
    =================================================
    
    This is the agent for getting telemetry information
    from the robots.

"""

helpmsg = """Telemetry Agent: Get telemetry data from the robots.
Possible commands are:
    tele all - returns the information for all robots
    tele raw - returns unformatted messages from robots
    tele "robot_id" - returns the information from one robot
    tele help - returns this message
    tele exit - exits the command agent"""
    
import queue
import logging
import socket
import json
from agentparent import AgentParent

# TeleAgent object
class TeleAgent(AgentParent):
    """ Telemetry Agent Object: allows reading robot telemetry
    """
    
    def __init__(self, config, name = ''):
        """ Constructor: Set up variables
        """
        self.name = name.lower()
        self.comqueue = queue.Queue() # Queue object for queries
        self.config = config # configuration
        self.log = logging.getLogger('Agent.'+self.name)
        self.exit = False # Indicates that loop should exit
        
    def __call__(self):
        """ Object call: Runs a loop that runs forever and handles
            user input.
        """
        ### Command loop
        while not self.exit:
            ### Look for task
            try:
                task, respqueue = self.comqueue.get(timeout = 0.1)
                task = task.strip()
            except queue.Empty:
                task = ''
            if len(task):
                self.log.debug('Got task <%s>' % task)
            else:
                continue
            retmsg = ''
            # Exit
            if 'exit' in task[:4].lower():
                self.exit = True
            # Help message
            elif 'help' in task[:4].lower():
                retmsg = helpmsg
            # Print all robot information
            elif 'all' in task[:3].lower():
                # Get the telemetry data
                s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                s.connect(self.config['sockets']['telesock'])
                s.sendall('botdata'.encode())
                data=s.recv(1024)
                s.close()
                # Print the inforamtion
                retmsg = '\n'
                teles = data.decode().split('\n')
                telej = json.loads(teles[0])
                for nam in telej:
                    retmsg += nam + '\t'
                retmsg += '\n'
                for tel in teles:
                    telej = json.loads(tel)
                    for nam in telej:
                        retmsg += str(telej[nam]) + '\t'
                    retmsg += '\n'
            # Print raw messages from robot telemetry
            elif 'raw' in task[:3].lower():
                                # Get the telemetry data
                s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                s.connect(self.config['sockets']['telesock'])
                s.sendall('botdata'.encode())
                data=s.recv(1024)
                s.close()
                retmsg = data.decode()
            else:
                pass
            # Send return message
            if len(retmsg):
                respqueue.put("%s: %s" % (self.name, retmsg))