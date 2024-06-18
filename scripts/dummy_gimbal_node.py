#!/usr/bin/env python
import rospy
from dji_osdk_ros.srv import GimbalAction, GimbalActionResponse, GimbalActionRequest
from dji_osdk_ros.srv import CameraStartShootSinglePhoto, CameraStartShootSinglePhotoResponse, CameraStartShootSinglePhotoRequest
from dji_osdk_ros.srv import CameraRecordVideoAction, CameraRecordVideoActionResponse, CameraRecordVideoActionRequest
from dji_osdk_ros.srv import CameraSetZoomPara, CameraSetZoomParaResponse, CameraSetZoomParaRequest


class DummyGimbalNode:

    def __init__(self):

        self.service_gimbal_action= rospy.Service('/gimbal_task_control', GimbalAction, self.gimbal_action_callback)
        self.service_camera_start_shoot_single_photo = rospy.Service('/camera_start_shoot_single_photo', CameraStartShootSinglePhoto, self.camera_start_shoot_single_photo_callback)
        self.service_camera_record_video_action = rospy.Service('/camera_record_video_action', CameraRecordVideoAction, self.camera_record_video_action_callback)
        self.service_camera_task_set_zoom_para= rospy.Service('/camera_task_set_zoom_para', CameraSetZoomPara, self.camera_task_set_zoom_para_callback)


    def gimbal_action_callback(self, req):
        rospy.loginfo('DummyGimbalNode: gimbal_action_callback')
        rospy.loginfo('DummyGimbalNode: is_reset={}, payload_index={}, rotationMode={}'.format(req.is_reset, req.payload_index, req.rotationMode))
        rospy.loginfo('DummyGimbalNode: pitch={}, roll={}, yaw={}, time={}'.format(req.pitch, req.roll, req.yaw, req.time))

        return GimbalActionResponse(req.pitch, req.roll, req.yaw, True)

    def camera_start_shoot_single_photo_callback(self, req):
        rospy.loginfo('DummyGimbalNode: camera_start_shoot_single_photo_callback')
        rospy.loginfo('DummyGimbalNode: payload_index={}'.format(req.payload_index))
        return CameraStartShootSinglePhotoResponse(True)

    def camera_record_video_action_callback(self, req):
        rospy.loginfo('DummyGimbalNode: camera_record_video_action_callback')
        rospy.loginfo('DummyGimbalNode: start_stop={}, payload_index={}'.format(req.start_stop, req.payload_index))
        return CameraRecordVideoActionResponse(True)

    def camera_task_set_zoom_para_callback(self, req):
        rospy.loginfo('DummyGimbalNode: camera_task_set_zoom_para_callback')
        rospy.loginfo('DummyGimbalNode: factor={}'.format(req.factor))
        return CameraSetZoomParaResponse(True)

if __name__ == '__main__':
    rospy.init_node('gimbal_node', anonymous=True)

    node = DummyGimbalNode()

    rospy.spin()