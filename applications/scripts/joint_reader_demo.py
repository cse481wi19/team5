#! /usr/bin/env python                                                                                 
                                                                                                       
import robot_api                                                                                       
import rospy       
import joint_state_reader      
                              
                       
def wait_for_time():   
    """Wait for simulated time to begin.
    """                
    while rospy.Time().now().to_sec() == 0:                                                            
        pass                                                                                           
                                                                                                       
                                                                                                       
def main():                                                                                            
    rospy.init_node('joint_reader_demo')                                                               
    wait_for_time()                                                                                    
    argv = rospy.myargv()                                                                              
    reader = joint_state_reader.JointStateReader()
    rospy.sleep(0.5)
    # Fetch Only
    # names = robot_api.ArmJoints.names()
    # Kuri: Browse joints and initialize your own names list
    names = ['eyelids_joint', 'head_1_joint', 'head_2_joint', 'wheel_left_joint', 'wheel_right_joint']
    arm_vals = reader.get_joints(names)
    for k, v in zip(names, arm_vals):
        print '{}\t{}'.format(k, v)
                      
                      
if __name__ == '__main__':
    main()