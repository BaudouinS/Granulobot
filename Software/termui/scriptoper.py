""" Control Program - Script Operator 
    =================================
    
    This operator runs scripts which are stored in files.
    The operator runs one script at a time until it ends
    or is paused.
    
    Command responses are forwarded to the client which
    sent the last command to the operator.
"""
helpmsg = """Scrip Operatpr: Runs scripts
    sciptname.txt : runs the specified script if the script is a relative
        filepathname, e.g. myscript.txt it is run out of the current
        scriptfolder. If it's an absolute filepathname
         e.g. /data/scripts/myscript.txt that file is run.
        the ".txt" can be omitted when calling a script
    status : returns the current scriptfolder and information on the current
        (most recent) scripts being run.
    stop : interrupts the current script. Both run and resume are valid
        commands after a stop
    resume : resume the most recent paused script
    scriptfolder /new/script/folder : set new scriptfolder
    
    Scripts can contain any command which would be accepted by the
    userinterface. An additional command pause seconds can be used.
    
    Command responses are forwarded to the most recent
    interface which communicated with this operator.
"""

import os
import time
import queue
from operatorparent import OperatorParent

class ScriptOper(OperatorParent):
    """ Script operator object: Runs scripts as commanded
        by other interfaces.
    """
    
    def __init__(self, config, name = ''):
        """ Constructor: Set up variables
        """
        super().__init__(config, name)
        # Current scriptfolder
        self.scriptfolder = ''
        # Status on running scripts (noscript, running, stopped, ended)
        self.status = 'noscript'
        # List of strings with loaded script
        self.cmdlist = []
        # Scriptpointer: index of next command to run in list
        self.cmdptr = 0
        # Waitime: unix time to wait until (used in wait command)
        self.waitime = time.time()-1.0
        # Scriptfile: filename of current / most recent script file
        self.scriptfile = ''
        self.answerqueue = queue.Queue() # Queue to answer to calling interface

    def __call__(self):
        """ Object call: Runs a loop forever, handles and
            issues querries
        """
        # Setup
        task = ''
        self.scriptfolder = os.path.expandvars(self.config['userinterface']['scriptfolder'])
        # loop
        while not self.exit:
            ### Check for and handle commands
            try:
                task, answerqueue = self.comqueue.get(block = False)
                task = task.strip()
            except queue.Empty:
                task = ''
            if len(task):
                self.log.debug('Got task <%s>' % task)
            ### Handle task
            #print(repr(self.comm))
            retmsg = ''
            # Exit
            if task.lower() == 'exit':
                self.exit = True
            # Help message
            elif task.lower() == 'help':
                retmsg = helpmsg
            # Status message
            elif task.lower() == 'status':
                retmsg = 'Status: ' + self.status
                retmsg += '\n    Scriptfile: ' + os.path.split(self.scriptfile)[-1]
                retmsg += '\n    Scriptfolder: ' + self.scriptfolder
                if self.status in ['running','paused']:
                    retmsg += '\n    Command Number: %d' % self.cmdptr
            elif task.lower() == 'stop':
                if self.status in ['running']:
                    self.status = 'stopped'
                    retmsg = 'Stopping script %s' % (os.path.split(self.scriptfile)[-1])
                else:
                    retmsg = 'No task running'
            elif task.lower() == 'resume':
                if self.status in ['running']:
                    retmsg = 'Script already running'
                elif self.status in ['ended']:
                    retmsg = 'No script to resume'
                else:
                    self.status = 'running'
                    retmsg = 'Resuming script %s' % (os.path.split(self.scriptfile)[-1])
            ### ADD MORE here
            # not a command - must be a script to run
            elif len(task):
                # Get scriptfile and make sure it exists
                newscript = os.path.join(self.scriptfolder,task)
                if os.path.exists(newscript + '.txt'):
                   newscript += '.txt' 
                if not os.path.exists(newscript):
                    retmsg = 'Invalid filename %s' % newscript
                # If a script is already running - return an error message
                elif self.status in ['running']:
                    retmsg = 'Unable to start script named %s, script %s is already running'
                    retmsg = retmsg % (os.path.split(self.scriptfile)[-1], task)
                else:
                    # Make full scriptfile (works even if full filepathname is given)
                    self.scriptfile = newscript
                    # Load script lines into self.cmdlist
                    # (remove commented out parts and make sure each line contains a command)
                    self.cmdlist = [ s.strip().split('#')[0] 
                                     for s in open(self.scriptfile,'rt')]
                    self.cmdlist = [ s for s in self.cmdlist if len(s)]
                    # Set cmdpointer
                    if len(self.cmdlist):
                        self.status = 'running'
                        self.cmdptr = 0
                        retmsg = 'started %s' % self.scriptfile
                        self.waitime = time.time()-1.0
                    else:
                        retmsg = 'Script %s has no commands'% self.scriptfile
                    # swap queues
                    if answerqueue != self.answerqueue:
                        tmp = answerqueue
                        answerqueue = self.answerqueue
                        self.answerqueue = tmp
            # Send return message
            if len(retmsg):
                answerqueue.put("%s: %s" % (self.name, retmsg))   
                if self.answerqueue != answerqueue:
                    self.answerqueue.put("%s: %s" % (self.name, retmsg))       
            # get responses from agents
            try:
                resp = self.respqueue.get(block = False)
                self.log.debug('Got response <%s>' % resp)
            except queue.Empty:
                resp = ''
            if len(resp):
                self.answerqueue.put(resp)
            # If time is up and running, process next command
            if self.status in ['running'] and self.waitime < time.time():
                if self.cmdptr < len(self.cmdlist):
                    cmd = self.cmdlist[self.cmdptr]
                    if 'wait' in cmd[:4]:
                        self.waitime = time.time() + float(cmd[4:].strip())
                    else:
                        self.sendtask(self.cmdlist[self.cmdptr])
                    self.cmdptr += 1
                else:
                    self.status = 'ended'
            time.sleep(0.001) # wait 1ms for queue's to catch up
