#include <sensyn_dji_gimbal_action/dji_gimbal_camera_action_client.h>


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

    double sign(double A){
        return (A>=0)-(A<0);
    }   

    bool sr::GimbalCameraActionClient::requestTaskAimCamera(const double roll, const double pitch, const double yaw, const double zoom, const double time) {

        sensyn_dji_gimbal_action::GimbalCameraGoal goal;
        goal.task_id = (int)(GimbalCameraActionType::AimCamera);
        goal.pitch = pitch;
        goal.roll = roll;
        goal.yaw = yaw;
        goal.zoom = zoom;
        goal.time = time;
        goal.is_reset = true;
        ROS_INFO ("requestTaskAimCamera: pitch = %f, roll = %f, yaw = %f, zoom = %f, time = %f", pitch, roll, yaw, zoom, time);

        sendGoal(goal);
        return true;
    }

    bool sr::GimbalCameraActionClient::requestTaskTargetCamera(const double roll, const double x, const double y, const double z, const double zoom, const double time) {

        sr::GimbalCameraActionClient::requestTaskTargetCamera(roll, x,y,z, zoom, time, true);

        return true;
    }
    bool sr::GimbalCameraActionClient::requestTaskTargetCamera(const double roll, const double x, const double y, const double z,
     const double zoom, const double time, const bool is_reset) {

        sensyn_dji_gimbal_action::GimbalCameraGoal goal;

        goal.task_id = (int)(GimbalCameraActionType::AimCamera);
        goal.roll = roll;
        goal.zoom = zoom;
        goal.time = time;
        goal.is_reset = is_reset;

        //spherical coordinate system to pitch and yaw
        double radius = sqrt(x*x + y*y + z*z);
        double epsilon = 0.001;
        if (radius < epsilon){
            goal.pitch = 0.;
            goal.yaw =  0.;
        }
        else if ( (x*x + y*y) < epsilon){
            goal.pitch = asin(z/radius) * 180 / M_PI;
            goal.yaw =  0.;
        }
        else{
            goal.pitch = asin(z/radius) * 180 / M_PI;
            goal.yaw =  sign(y) * acos(x / sqrt(x*x + y*y)) * 180 / M_PI;
        }
             

        std::cout << "pitch = "<<goal.pitch << ", roll = "<<goal.roll << ",. yaw = "<<goal.yaw <<std::endl;

        sendGoal(goal);

        return true;
    }


    bool sr::GimbalCameraActionClient::requestTaskTakePicture(const string camera_setting) {

        sensyn_dji_gimbal_action::GimbalCameraGoal goal;
        goal.task_id = (int)(GimbalCameraActionType::TakePicture);
        goal.camera_setting = camera_setting;
        ROS_INFO ("requestTaskTakePicture: camera_setting = %s", camera_setting.c_str());

        sendGoal(goal);
        return true;
    }

    bool sr::GimbalCameraActionClient::requestTaskRecordVideo(const string camera_setting) {
        //coming soon...

        return true;

    }


    bool sr::GimbalCameraActionClient::requestTaskStartRecordVideo(const string camera_setting) {
        sensyn_dji_gimbal_action::GimbalCameraGoal goal;
        goal.task_id = (int)(GimbalCameraActionType::StartRecordVideo);
        goal.camera_setting = camera_setting;
        ROS_INFO ("requestTaskStartRecordVideo: camera_setting = %s", camera_setting.c_str());

        sendGoal(goal);
        return true;
    }


    bool sr::GimbalCameraActionClient::requestTaskStopRecordVideo() {
        sensyn_dji_gimbal_action::GimbalCameraGoal goal;
        goal.task_id = (int)(GimbalCameraActionType::StopRecordVideo);
        ROS_INFO ("requestTaskStopRecordVideo");

        sendGoal(goal);
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

