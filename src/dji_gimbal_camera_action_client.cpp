#include <dji_gimbal_camera_action_client.h>


// class GimbalCameraActionClient
// {
//     public:
//     actionlib::SimpleActionClient<sensyn_dji_gimbal_action::GimbalCameraAction> client;
        // using actionlib::SimpleActionClient;


        // void doneCb(const SimpleClientGoalState &state, RobotActionResult &result) {
        //     if (state.state_ == state.SUCCEEDED) {
        //         //robot_action_controller::RobotActionResult ret;
        //         result.done = true;

        //     } else {
        //         ROS_INFO("Task failed!");
        //     }
        // }

    bool sr::GimbalCameraActionClient::requestTaskAimCamera(const double roll, const double pitch, const double yaw, const double zoom) {

        sensyn_dji_gimbal_action::GimbalCameraGoal goal;
        goal.task_id = int(RobotAction_Type::AimCamera);
        goal.pitch = pitch;
        goal.roll = roll;
        goal.yaw = yaw;
        goal.zoom = zoom;

        sendGoal(goal);
        waitForResult();
        return true;
    }

    bool sr::GimbalCameraActionClient::requestTaskTargetCamera() {

        //coming soon...

        return true;
    }


    bool sr::GimbalCameraActionClient::requestTaskTakePicture(const string camera_setting) {

        sensyn_dji_gimbal_action::GimbalCameraGoal goal;
        goal.task_id = int(RobotAction_Type::TakePicture);
        goal.camera_setting = camera_setting;

        sendGoal(goal);
        waitForResult();
        return true;
    }

    bool sr::GimbalCameraActionClient::requestTaskRecordVideo(const string camera_setting) {
        //coming soon...

        return true;

    }


    bool sr::GimbalCameraActionClient::requestTaskStartRecordVideo(const string camera_setting) {
        sensyn_dji_gimbal_action::GimbalCameraGoal goal;
        goal.task_id = int(RobotAction_Type::StartRecordVideo);
        goal.camera_setting = camera_setting;

        sendGoal(goal);
        waitForResult();
        return true;
    }


    bool sr::GimbalCameraActionClient::requestTaskStopRecordVideo() {
        sensyn_dji_gimbal_action::GimbalCameraGoal goal;
        goal.task_id = int(RobotAction_Type::StopRecordVideo);

        sendGoal(goal);
        waitForResult();
        return true;
    }

    
// };

// int main (int argc, char** argv)
// {
//   // Initialize ROS
//   ros::init (argc, argv, "dji_gimbal_action_client");

//   GimbalCameraActionClient client();
//   ros::spin();

//   return 0;
// }

