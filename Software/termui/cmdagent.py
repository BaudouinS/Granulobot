""" Interface Agent Control Program - Command Agent
    =================================================
    
    This is the agent for sending commands to the robots.

"""

helpmsg = """Command Agent: Send commands to the robots.
Possible commands are:
    cmd "robot_id" "command" - send command to a robot
    cmd help - returns this message
    cmd exit - exits the command agent"""
    
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
            # Exit
            if 'exit' in task[:4].lower():
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