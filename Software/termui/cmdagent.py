""" Interface Agent Control Program - Command Agent
    =================================================
    
    This is the agent for sending commands to the robots.

"""

helpmsg = """Command Agent: Send commands to the robots.
Possible commands are:
    cmd <robot_id> <command> - send command to a robot
    cmd all <command> - sends the command to all robots
    cmd <ip address> <command> - sends the command to specified IP address
        Boradcast is sent if the IP address ends with .255
    cmd help - returns this message
    cmd exit - exits the command agent
    cmd quitele - sends shutdown command to GBOT_TELEMETRY program
The < > brackets indicate text needed to be entered and are not
necessary. It is possible to send a command to multiple bots by listing
multiple bot IDs or IP addresses separated with "/".
"""

import queue
import logging
import socket
import json
from agentparent import AgentParent
from gbotlib import gbutils

# TeleAgent object
class CmdAgent(AgentParent):
    """ Command Agent: send commands to robots
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
            # Quit telemetry program
            if 'quitele' in task[:7].lower():
                retmsg = 'Shutting down gbot_telemetry'
                gbutils.sendcommand(self.config,'exit')
            # Exit
            elif 'exit' in task[:4].lower():
                self.exit = True
            # Help message
            elif 'help' in task[:4].lower():
                retmsg = helpmsg
            # Print all robot information
            else:
                # Send command
                gbutils.sendcommand(self.config,task)
            # Send return message
            if len(retmsg):
                respqueue.put("%s: %s" % (self.name, retmsg))
