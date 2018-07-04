"""
@author: Luca Muratore (luca.muratore@iit.it)
"""

import rospy
import time
import rosservice

from XBotCore.srv import cmd_service


class CommandListener:

    def __service_callback(self, req):
        self.new_data = True
        self.last_command = req.cmd
        return True

    def __init__(self, service_name):

        self.service_name = service_name
        self.last_command = ""
        self.new_data = False

        service_list = rosservice.get_service_list()
        if not '/' + service_name in service_list:
            self.service = rospy.Service(service_name, cmd_service,  self.__service_callback)
        print "Listening to command"

    def __del__(self):
        self.service.shutdown()

    def get_last_command(self):
        while (self.new_data == False):
            time.sleep(0.5)

        self.new_data = False
        return self.last_command

