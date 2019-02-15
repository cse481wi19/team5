#!/usr/bin/env python                                                                                  
                                                                                                       
import rospy
from sensor_msgs.msg import JointState                                                                                           
                                                                                                       
class JointStateReader(object):                                                                        
    """Listens to /joint_states and provides the latest joint angles.                                  
                                                                                                       
    Usage:                                                                                             
        joint_reader = JointStateReader()                                                              
        rospy.sleep(0.1)                                                                               
        joint_reader.get_joint('shoulder_pan_joint')                                                   
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])                         
    """                                                                                                
    def __init__(self):                                                                                
        self.sub = None
        self.alljoints = ['eyelids_joint', 'head_1_joint', 'head_2_joint', 'wheel_left_joint', 'wheel_right_joint']
        self.jointpositon = [None for x in range(5)]                                                                                 
        self.jointstate = dict(zip(self.alljoints, self.jointpositon))

    def callback(self, data):
        self.jointpositon = data.position
        self.jointstate = dict(zip(self.alljoints, self.jointpositon))
        # rospy.logerr(self.jointstate)
        return self.jointstate

    def isNone(self):
        if None in self.jointpositon:
            return True

    def get_joint(self, name):                                                                         
        """Gets the latest joint value.                                                                
                                                                                                       
        Args:                                                                                          
            name: string, the name of the joint whose value we want to read.                           
                                                                                                       
        Returns: the joint value, or None if we do not have a value yet.                               
        """          
        self.sub = rospy.Subscriber("/joint_states", JointState, self.callback)
        while self.isNone():
            pass 
        js = self.jointstate[name]                                                                                 
        return js                                                                     
                                                                                                       
    def get_joints(self, names):                                                                       
        """Gets the latest values for a list of joint names.                    
                                                                                
        Args:                                                                   
            name: list of strings, the names of the joints whose values we want 
                to read.                                                        
                                                                                
        Returns: A list of the joint values. Values may be None if we do not    
            have a value for that joint yet.                                    
        """ 
        self.sub = rospy.Subscriber("/joint_states", JointState, self.callback)
        while self.isNone():
            pass        
        js = []
        for name in names:
            js.append(self.jointstate[name])                          
        return js