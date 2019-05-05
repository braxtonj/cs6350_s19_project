#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

if __name__ == '__main__':
    rospy.init_node('parameter_reloader')

    # TODO need to get rid of hard-coded name for service
    
    try:
        reload_params = rospy.ServiceProxy("/lbr4_pd_control/reload_parameters", Empty)
        success = reload_params()
    except rospy.ServiceException, e:
        rospy.logerr("Service call to reload parameters failed.")

    if success:
        rospy.loginfo("Service call to reload parameters was successful!")
