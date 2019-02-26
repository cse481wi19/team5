#! /usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo, RegionOfInterest
from std_msgs.msg import Header

class CamInfo(object):
    def __init__(self):
        header = Header(frame_id='upward_looking_camera_link')
        self._cam_info = \
        CameraInfo(
                    header=header,
                    height=480,
                    width=640,
                    distortion_model='plumb_bob',
                    D=[0.0, 0.0, 0.0, 0.0, 0.0],
                    K=[504.78216005824515, 0.0, 320.5, 0.0, 504.78216005824515, 240.5, 0.0, 0.0, 1.0],
                    R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    P=[504.78216005824515, 0.0, 320.5, -0.0, 0.0, 504.78216005824515, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0],
                    binning_x=0,
                    binning_y=0,
                    roi=RegionOfInterest(x_offset=0,
                                        y_offset=0,
                                        height=0,
                                        width=0,
                                        do_rectify=False
                                        )
                    )
        self._pub = rospy.Publisher('/upward_looking_camera/camera_info/fake',CameraInfo,queue_size=10)

    def pubCamInfo(self):
        rospy.init_node('fake_cam_info_node', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        _seq = 0
        while not rospy.is_shutdown():
            _seq += 1
            _header = Header(
                frame_id='upward_looking_camera_link',
                stamp=rospy.Time.now(),
                seq=_seq
                )
            self._cam_info.header = _header
            self._pub.publish(self._cam_info)
            rate.sleep()

if __name__ == '__main__':
    caminfo = CamInfo()
    try:
        caminfo.pubCamInfo()
    except rospy.ROSInterruptException:
        pass

