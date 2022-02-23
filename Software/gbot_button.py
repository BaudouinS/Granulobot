""" Granular Robots Stop Button

    This program provides a single button that runs one command.
    
    Gbot_telemetry has to be running.
"""

from tkinter import *
import tkinter.font as font
from gbotlib import gbutils

def cmdrun():
    gbutils.sendcommand(config,'all ' + config['button']['cmd'])
    print('sent %s to all robots' % config['button']['cmd'])


# Load configuration
config = gbutils.setconfig(sys.argv)
# Make the window
master = Tk()
master.geometry('250x150')
# Make the button
lgfont = font.Font(size = 48)
btn = Button(master, text = config['button']['text'], bd = '10',
             command = cmdrun)
btn['font'] = lgfont
btn.pack(fill = BOTH, expand = True)
# Run mainloop
master.mainloop()