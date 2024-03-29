""" Interface Agent Control Program - Configuration Agent
    =====================================================
    
    This is the agent for reading and changing the current 
    system configuration.

"""

helpmsg = """Configuration Agent: Read and edit current system configuration.
Possible commands are:
    conf all - returns the configuration
    conf sectionlist - returns list of all configuration sections
    conf "section" - returns the contents of a particular config section
    conf "section" "keyword" - returns the value of a particular keyword
    conf "section" "keyword" "value" - sets the value for that particular keyword
    conf help - returns this message
    conf exit - exits the command agent"""
    
import queue
import logging
from io import StringIO
from agentparent import AgentParent


# ConfigAgent Object
class ConfigAgent(AgentParent):
    """ Config object: allows interaction with config
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
        """ Object call: Runs a loop that runs forever and forwards
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
            #print(repr(self.comm))
            retmsg = ''
            # Exit
            if 'exit' in task[:4].lower():
                self.exit = True
            # Help message
            elif 'help' in task[:4].lower():
                retmsg = helpmsg
            # Print all config
            elif 'all' in task[:3].lower():
                #sio = StringIO('')
                #self.config.write(sio)
                #retmsg = sio.getvalue()
                #sio.close()
                retmsg = repr(self.config)
            # Print section list
            elif 'sectionlist' in task[:11].lower():
                retmsg = 'List of config sections'
                for k in self.config.keys():
                    retmsg += '\n' + k
            # Check if it's just sectionname -> print section
            elif ' ' not in task:
                sect = task.strip()
                if sect in self.config:
                    # if it's a dict - print contents
                    if isinstance(self.config[sect], dict):
                        retmsg = '[%s]' % sect
                        for k in self.config[sect]:
                            retmsg += '\n    %s = %s' % (k, self.config[sect][k]) 
                    else:
                        retmsg = '%s = %s' % (sect, self.config[sect])
                else:
                    retmsg = 'Invalid section: %s' % sect
            # We have at least section and key
            else:
                # Get section
                sect, task = task.split(' ',1)
                if sect in self.config:
                    # If no further word, return value
                    if ' ' not in task:
                        key = task.strip()
                        if not key in self.config[sect]:
                            retmsg = 'Invalid key=%s in section=%s' % (sect,key)
                        else:
                            retmsg = self.config[sect][key]
                    else:
                        key, val = task.split(' ',1)
                        self.config[sect][key] = val
                        retmsg = 'set [%s] %s = %s' % (sect, key, val)
                else:
                    retmsg = 'Invalid section: %s' % sect
            # Send return message
            if len(retmsg):
                respqueue.put("%s: %s" % (self.name, retmsg))
        