#include <ros/ros.h>
#include "sensyn_dji_gimbal_action/dji_gimbal_camera_action_client.h"

using namespace actionlib;
using namespace sensyn_dji_gimbal_action;
using namespace sr;


bool waitResult_gimbal_camera(double wait_duration, sr::GimbalCameraActionClient& gimbal_camera_action_client) {
    sensyn_dji_gimbal_action::GimbalCameraResult result;
    bool wait_resullt = false;
    while (!wait_resullt){

        wait_resullt = gimbal_camera_action_client.waitForResult(ros::Duration(0, (int) (1000000000 * wait_duration)));
        result = *gimbal_camera_action_client.getResult();
        ros::spinOnce();
    }
    return result.done;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_dji_gimbal_camera_action_client");
    // ノードハンドラの作成
    ros::NodeHandle nh;

    // GimbalCameraActionClientのインスタンスを作成
    sr::GimbalCameraActionClient gimbal_camera_action_client("dji_gimbal_camera_action_server", true);

    // サーバーが立ち上がるのを待つ
    while (!gimbal_camera_action_client.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the dji_gimbal_camera_action_server");
    }

    bool result;

    //test requestTaskAimCamera
    ROS_INFO("test requestTaskAimCamera");
    result = gimbal_camera_action_client.requestTaskAimCamera(0., 0., 0., 5., 0.2);
    waitResult_gimbal_camera(0.5, gimbal_camera_action_client);

    //test requestTaskTargetCamera
    ROS_INFO("test requestTaskTargetCamera");
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 1.0;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.0;
    double zoom = 2.0;
    double time = 2.0;
    bool is_reset = true;
    result = gimbal_camera_action_client.requestTaskTargetCamera(0., target_pose.position.x, target_pose.position.y, target_pose.position.z, zoom, time, is_reset);
    waitResult_gimbal_camera(0.5, gimbal_camera_action_client);
    
    //test requestTaskTakePicture
    ROS_INFO("test requestTaskTakePicture");
    result = gimbal_camera_action_client.requestTaskTakePicture("");
    waitResult_gimbal_camera(0.5, gimbal_camera_action_client);

    //test requestTaskStartRecordVideo
    ROS_INFO("test requestTaskStartRecordVideo");
    result = gimbal_camera_action_client.requestTaskStartRecordVideo("");
    waitResult_gimbal_camera(0.5, gimbal_camera_action_client);

    //test requestTaskStopRecordVideo
    ROS_INFO("test requestTaskStopRecordVideo");
    result = gimbal_camera_action_client.requestTaskStopRecordVideo();
    waitResult_gimbal_camera(0.5, gimbal_camera_action_client);

    
    ros::spin();

    return 0;
}



