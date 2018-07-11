#!/usr/bin/env python


import rospy
from std_srvs.srv import SetBool
from XBotCore.srv import status_service
from cartesian_interface.srv import LoadController
from centauro_tools.msg import HeriHandControl
    
from mttkinter import mtTkinter as tk
import ttk
import time
from threading import Thread
import random
import Queue

import os
import signal
import subprocess


################# callbacks

def about():
   tkMessageBox.showinfo("PHOLUS Demo", "Demo for PHOLUS D3 Meeting")

def switch_plugin(name, switch):
    if switch == 1 :
        print "Starting " + name 
    elif switch == 0 :
        print "Stopping " + name 
    else :
        print "ERROR: " + switch + " is not supported when starting/stopping plugin " + name
        return False
    rospy.wait_for_service(name + '_switch', timeout=1.0)
    try:
        plugin_switch = rospy.ServiceProxy(name + '_switch', SetBool)
        return plugin_switch(switch)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def callback():     
    print "callback!"
    
    
######################################

class CheckPluginStatus(object):

    def __init__(self, obj, plugin, row_num, interval=1):

        
        self.obj = obj
        self.plugin = plugin
        self.row_num = row_num
        self.interval = interval
        
        # send a default message
        self.label_string = 'Waiting for status ...'
        msg = str(self.row_num) + "--" + self.label_string
        self.obj.queue.put(msg)

        thread = Thread(target=self.run, args=())
        thread.daemon = True                            # Daemonize thread
        thread.start()                                  # Start the execution

        #time.sleep(0.1)
        
    def run(self):
        """ Method that runs forever """
        while True:

            rospy.wait_for_service(self.plugin + '_status')

            plugin_status = rospy.ServiceProxy(self.plugin + '_status', status_service)
            res = plugin_status()
            
            self.label_string = res.status
            msg = str(self.row_num) + "--" + self.label_string
            self.obj.queue.put(msg)
  
            time.sleep(self.interval)

######################################           

def about():
   tkMessageBox.showinfo("PHOLUS Demo", "Demo for PHOLUS D3 Meeting")

def create_plugin_control(master,plugin_name, row_num, status_label):
    tk.Label(master, text=plugin_name, font='Helvetica 14 bold').grid(row=row_num,column=0, ipadx=20, ipady=20)
    tk.Button(master, text="Start", command=lambda: switch_plugin(plugin_name, 1)).grid(row=row_num,column=1)
    tk.Button(master, text="Stop", command=lambda: switch_plugin(plugin_name, 0)).grid(row=row_num,column=2)
    l = tk.Label(master, text="")
    l.grid(row=row_num,column=3, ipadx=10)
    l.config(bg='orange')
    status_label[row_num] = l
    
def cartesian_interface():
    # start the cartesian_interface server in the background
    my_env = os.environ.copy()
    subprocess.Popen(["roslaunch", "cartesian_interface", "cartesian_server.launch", "use_xbot_config:=true"], env=my_env)

def load_controller(controller):
    rospy.wait_for_service('/xbotcore/cartesian/load_controller', timeout=1.0)
    try:
        ci_load = rospy.ServiceProxy('/xbotcore/cartesian/load_controller', LoadController)
        res = ci_load(controller)
        print res.message
        
        # restart markers and joypads TBD do it properly
        subprocess.Popen(["killall", "-9", "marker_spawner"])
        subprocess.Popen(["rosrun", "cartesian_interface", "marker_spawner"])
        subprocess.Popen(["killall", "-9", "joystick_spawner"])
        subprocess.Popen(["rosrun", "cartesian_interface", "joystick_spawner"])
        
        return res.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
        
def xbot():
     subprocess.Popen(["XBotCore", "-DV"])

def heri_control(primitive, percentage):
    pub = rospy.Publisher('/heri_hand_control', HeriHandControl, queue_size=10)
    heri_ctrl = HeriHandControl()
    heri_ctrl.primitive = primitive
    heri_ctrl.percentage = float(percentage.get())
    print heri_ctrl
    pub.publish(heri_ctrl)
    


class GuiPart:
    def __init__(self, master, queue, endCommand):
        self.queue = queue
        
        self.status_label = dict()
        
        #Set up the GUI
        r = -1
        
        # XBotCore Dummy
        r += 1
        tk.Button(master, text='Run XBotCore', font='Helvetica 18 bold', command=xbot).grid(row=r, column=0,pady=20)
        
        # plugins
        r += 1
        create_plugin_control(master,'HomingExample', r, self.status_label)
        r += 1
        create_plugin_control(master,'XBotCommunicationPlugin', r, self.status_label)
        r += 1
        create_plugin_control(master,'HeriHand', r, self.status_label)
        
        # HeriHand control
        percentage = tk.StringVar(root, value='0.0')
        #percentage.trace("w", lambda name, index, mode, percentage=percentage: callback(percentage))

        r += 1
        
        tk.Button(master, text="Grasp", font='Calibri 12', command=lambda: heri_control("grasp", percentage)).grid(row=r, column=1, pady=20)
        tk.Button(master, text="Pinch", font='Calibri 12', command=lambda: heri_control("pinch", percentage)).grid(row=r, column=2)
        tk.Button(master, text="Tool Grasp", font='Calibri 12', command=lambda: heri_control("tool_grasp", percentage)).grid(row=r, column=3)
        tk.Button(master, text="Tool Trigger", font='Calibri 12', command=lambda: heri_control("tool_trigger", percentage)).grid(row=r, column=4)
        
        r += 1
        
        tk.Label(master, text="Closure (0.0 to 1.0)",  font='Calibri 12',).grid(row=r, column=1, sticky=tk.W, padx=20, pady=20)

        e1 = tk.Entry(master, textvariable=percentage)
        e1.grid(row=r, column=2)

        #e1 = Entry(master)
        #e2 = Entry(master)
        
        #  CI
        b = tk.Button(master, text='Cartesian Interface', font='Helvetica 12 bold', command=lambda: cartesian_interface())
        r += 1
        b.grid(row=r, column=0, rowspan=2, pady=20)
        b.config(bg='cyan')
        
        v = tk.IntVar()
        c1 = tk.Radiobutton(master, text='WheeledMotion', font='Helvetica 12', variable=v, value=0, 
                            command=lambda: load_controller('WheeledMotion'))
        c1.grid(row=r, column=1, pady=20, sticky=tk.W)
        
        r += 1
        c2 = tk.Radiobutton(master, text='OpenSot', font='Helvetica 12', variable=v, value=1, 
                            command=lambda: load_controller('OpenSot'))
        c2.grid(row=r, column=1, sticky=tk.W)

        #Quit
        tk.Button(master, text='Quit', font='Helvetica 12', command=endCommand).grid(column=0,pady=20)

    def processIncoming(self):
        """Handle all messages currently in the queue, if any."""
        while self.queue.qsize(  ):
            try:
                msg = self.queue.get(0)
                #print msg
                
                id =  int(msg.split("--")[0])
                status = msg.split("--")[1]
                            
                self.status_label[id].config(text=status)
                if status == 'STOPPED':
                    self.status_label[id].config(bg='red')
                elif status == 'RUNNING':
                    self.status_label[id].config(bg='green')
            except Queue.Empty:
                # just on general principles, although we don't
                # expect this branch to be taken in this case
                pass

class ThreadedClient:
    """
    Launch the main part of the GUI and the worker thread. periodicCall and
    endApplication could reside in the GUI part, but putting them here
    means that you have all the thread controls in a single place.
    """
    def __init__(self, master):
        """
        Start the GUI and the asynchronous threads. We are in the main
        (original) thread of the application, which will later be used by
        the GUI as well. We spawn a new thread for the worker (I/O).
        """
        self.master = master

        # Create the queue
        self.queue = Queue.Queue(  )

        # Set up the GUI part
        self.gui = GuiPart(master, self.queue, self.endApplication)

        # Set up the thread to do asynchronous I/O
        # More threads can also be created and used, if necessary
        self.running = 1
        # TBD AVOID CODE DUPLICATION HACK
        CheckPluginStatus(self, "HomingExample", 1)
        CheckPluginStatus(self, "XBotCommunicationPlugin", 2)
        CheckPluginStatus(self, "HeriHand", 3)

        # Start the periodic call in the GUI to check if the queue contains
        # anything
        self.periodicCall(  )

    def periodicCall(self):
        """
        Check every 200 ms if there is something new in the queue.
        """
        self.gui.processIncoming(  )
        if not self.running:
            # This is the brutal stop of the system. You may want to do
            # some cleanup before actually shutting it down.
            import sys
            sys.exit(1)
        self.master.after(200, self.periodicCall)

    def endApplication(self):
        self.running = 0
        # kill subprocesses
        os.killpg(0, signal.SIGINT)


# root view
root = tk.Tk()

root.style = ttk.Style()
#('clam', 'alt', 'default', 'classic')
root.style.theme_use("clam")

root.title('PHOLUS demo')
root.minsize(1080,720)
root.geometry("720x480")

rospy.init_node('PHOLUS_demo', anonymous=True)

client = ThreadedClient(root)
root.mainloop()    
