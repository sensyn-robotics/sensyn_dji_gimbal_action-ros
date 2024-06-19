#!/usr/bin/env python
import rospy
from dji_osdk_ros.srv import GimbalAction, GimbalActionResponse, GimbalActionRequest
from dji_osdk_ros.srv import CameraStartShootSinglePhoto, CameraStartShootSinglePhotoResponse, CameraStartShootSinglePhotoRequest
from dji_osdk_ros.srv import CameraRecordVideoAction, CameraRecordVideoActionResponse, CameraRecordVideoActionRequest
from dji_osdk_ros.srv import CameraSetZoomPara, CameraSetZoomParaResponse, CameraSetZoomParaRequest

import time

import rosunit
import unittest

class Test_gimbal(unittest.TestCase):

    # constractor
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # test variables
        self.called_gimbal_action_reset = False
        self.called_gimbal_action = False
        self.called_camera_start_shoot_single_photo = False
        self.called_camera_record_video_action_start = False
        self.called_camera_record_video_action_stop = False
        self.called_camera_task_set_zoom_para = False

        # service
        rospy.init_node('test_gimbal_server')
        self.service_gimbal_action= rospy.Service('/gimbal_task_control', GimbalAction, self.gimbal_action_callback)
        self.service_camera_start_shoot_single_photo = rospy.Service('/camera_start_shoot_single_photo', CameraStartShootSinglePhoto, self.camera_start_shoot_single_photo_callback)
        self.service_camera_record_video_action = rospy.Service('/camera_record_video_action', CameraRecordVideoAction, self.camera_record_video_action_callback)
        self.service_camera_task_set_zoom_para= rospy.Service('/camera_task_set_zoom_para', CameraSetZoomPara, self.camera_task_set_zoom_para_callback)
        
        # wait 10 sec
        rospy.sleep(30)

    def gimbal_action_callback(self, req): 
        if req.is_reset:
            self.called_gimbal_action_reset = True
        else:
            self.called_gimbal_action = True

        return GimbalActionResponse(req.pitch, req.roll, req.yaw, True)

    def camera_start_shoot_single_photo_callback(self, req):
        self.called_camera_start_shoot_single_photo = True

        return CameraStartShootSinglePhotoResponse(True)

    def camera_record_video_action_callback(self, req):
        if req.start_stop == 1:
            self.called_camera_record_video_action_start = True
        else:
            self.called_camera_record_video_action_stop = True

        return CameraRecordVideoActionResponse(True)

    def camera_task_set_zoom_para_callback(self, req):
        self.called_camera_task_set_zoom_para = True
        return CameraSetZoomParaResponse(True)

    def test_gimbal(self):

        self.assertTrue(self.called_gimbal_action_reset)
        self.assertTrue(self.called_gimbal_action)
        self.assertTrue(self.called_camera_start_shoot_single_photo)
        self.assertTrue(self.called_camera_record_video_action_start)
        self.assertTrue(self.called_camera_record_video_action_stop)


if __name__ == '__main__':
    rosunit.unitrun("sensyn_dji_gimbal_action", 'test_gimbal_server', Test_gimbal)
