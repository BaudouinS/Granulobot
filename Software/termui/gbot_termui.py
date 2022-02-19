""" Granular Robots User Interface Program - Controller
    ===================================================
    
    This program provides a simple user interface for Granular Robots
    project. It has the following functions:
    - Provide a user interface to allow local access to all functions
    - Get Telemetry data from the robots
    - Send commands to the robot
    
    Usage: python gbot_termui.py conigfile.txt
    
    Process: The programs has interfaces (user, web, port) which send
        tasks to agents (telemetry, command). Each interface and agent has its
        own thread. Each agent has a input task queue where it gets
        a string for the task and a queue to respond to (optional).
        Generally interfaces expect one response from an agent for each task.

    Author: Marc Berthoud - mgb11@cornell.edu

"""

### Preparation

# Imports
import os
import sys
import queue
import time
import logging
import threading
import configparser
from distutils.command.config import config
from agentparent import AgentParent
from interparent import InterParent
from userinterface import InterUser
from scriptoper import ScriptOper
from configagent import ConfigAgent
from teleagent import TeleAgent
from cmdagent import CmdAgent
from gbotlib import gbutils

def gbotcontrol():
    """ Run the GBot control
    """
    # Load config file
    config = gbutils.setconfig(sys.argv)
    # Setup logging
    gbutils.setuplogging(config)
    logging.getLogger('TermUI').info('Terminal User Interface: Starting UP')
    # Make interfaces and agents
    inusr = InterUser(config, 'User')
    opscr = ScriptOper(config, 'Script')
    agconf = ConfigAgent(config, 'Conf')
    agtele = TeleAgent(config, 'Tele')
    agcmd = CmdAgent(config,'Cmd')

    # Register agents with interfaces
    for agent in [agconf, agtele, agcmd, opscr]:
        inusr.addagent(agent)
        opscr.addagent(agent)

    # Run items as threads (as daemons such that they shut down on exit)
    threads = {}
    for item in [agconf, agtele, agcmd, opscr, inusr]:
        thread = threading.Thread(target = item)
        thread.daemon = True
        thread.start()
        threads[item.name] = thread
    # Wait and do some stuff
    time.sleep(0.5)
    # Join with User Interface thread
    threads['User'].join()

if __name__ == '__main__':
    """ Main function for calling command line. Passes the configuration file
        name to the control function
    """
    # Check input
    if len(sys.argv) < 2:
        print("""Usage:
    python controll.py configfile.txt
where
    configfile.txt has to be the filepathname for a valid config file
""")
        exit()
    # Get config file name
    Config_FilePathName = sys.argv[1]
    # Call HWP control
    gbotcontrol()
    print("That's All Folks!")
