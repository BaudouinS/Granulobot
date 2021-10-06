""" Granular Robot Library 

    Utility functions for the granular robot project
    
"""
# Imports
import os
import sys
import configobj

def setconfig(config = None):
    """ Loads the given config file and returns the config object.
    
        If a list of files is given, the first connfiguration is
        loaded and further configurations are loaded overwriting
        already existing values. This allows the use of baseconfig
        and additional delta configs.
        
        If a config object is given, that object is returned.
        
        The function will skip config files that is is unable to
        load. Failed files are listed in setconfigmessage (see
        below).
        
        This makes it possible to pass sys.argv to setconfig
        as invalid filenames and files that are not config files
        are skipped.
        
        After loading the config files, the function sets
        environment variables if an [envars] section exits in
        the configruration.
        
        The function writes a message to the setconfigmessage
        value of the returned configuration.
    """
    confloaded = None
    retmsg = ''
    ### Check if a config object is given
    if isinstance(config,configobj.ConfigObj):
        # if config is a ConfObj -> set it
        confloaded = config
    else:
        ### Get list of files to load
        # It's a string -> check if file exists
        if isinstance(config, str):
            if os.path.exists(config):
                config = [config]
            # Else no config is set
            else:
                config = None
        # It's a list, pass along all valid filenames
        elif isinstance(config, (list, tuple)):
            config = [ conf for conf in config if os.path.isfile(conf)]
            # If there are no valid filenames -> no config is set
            if len(config) < 1: config = None
        # If it's something else raise an error
        elif config:
            raise ValueError("setconfig: Invalid config = %s" % repr(config))
        # If no config is loaded look for it in sys.argv
        #if not config:
        #    retmsg += '\nSearching sys.argv for config files'
        #    config = [ conf for conf in sys.argv if os.path.isfile(conf)]
        #    # If there are no valid filenames -> no config is set
        #    if len(config) < 1: config = None
        if not config:
            raise RuntimeError('setconfig: No config files given')
        ### Load config files
        # Get baseconfig (first loading file)
        while len(config):
            try:
                confloaded = configobj.ConfigObj(config[0])
                break
            except:
                retmsg += '\nunable to load %s' % config[0]
                config.pop(0)
        if len(config) < 1:
            raise RuntimeError('setconfig: No valid confiles: %s' % retmsg.strip())
        retmsg += 'baseconf = %s' % config[0]
        config.pop(0)
        # Get delta configs
        while len(config):
            # Open the delta config
            try:
                delconf = configobj.ConfigObj(config[0])
            except:
                retmsg += '\nunable to load %s' % config[0]
                config.pop(0)
                continue
            # Merge the delta config into existing config
            confloaded.merge(delconf)
            retmsg += '\ndeltaconf = %s' % config[0]
            config.pop(0)
    if not confloaded:
        raise RuntimeError('setconfig: unable to load configuration')
    ### Set environment variables
    if 'envars' in confloaded:
        for var in confloaded['envars']:
            os.environ[var] = str( confloaded['envars'][var] )
    ### Finish up
    confloaded['setconfigmessage'] = retmsg.strip()
    return confloaded
