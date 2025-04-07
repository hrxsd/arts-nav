# This example demonstrates how to use LCM with the Python select module


import os
import sys
root_path = os.path.dirname(os.path.abspath(__file__))
print(root_path)
root_path_1 = '/'.join(root_path.split('/')[:-1])
sys.path.append(root_path_1)

import numpy as np
import rospy
from navigation.srv import lcm_srv,lcm_srvRequest,lcm_srvResponse

import select
import lcm
from exlcm import example_t
# import example_t



class lcm_server:
    def __init__(self):
        self.lc = lcm.LCM()
        self.lc.subscribe("Cook_done", self.my_handler)
        self.cook_done = False

    def my_handler(self, channel, data):
        msg = example_t.decode(data)
        print("Received message on channel \"%s\"" % channel)
        print("   enabled     = %s" % str(msg.enabled))
        self.cook_done = msg.enabled
        print("")
        
    def lcm_do(self, req):
        lcm_res = lcm_srvResponse()
        while True:
            try:
                timeout = 1.5  # amount of time to wait, in seconds
                while True:
                    if self.cook_done:
                        lcm_res.cook_done = True
                        return lcm_res
                    rfds, wfds, efds = select.select([self.lc.fileno()], [], [], timeout)
                    if rfds:
                        self.lc.handle()
                    else:
                        print("Waiting for message...")
            except KeyboardInterrupt:
                pass

        return 

if __name__ == '__main__':
    try:
        rospy.init_node('lcm_service_node')
        lcm_sev = lcm_server()
        rospy.Service('lcm_server', lcm_srv, lcm_sev.lcm_do)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    