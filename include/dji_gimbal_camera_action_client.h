// #ifndef ELAS_ROBOT_ACTION_CONTROL_CLIENT_H
// #define ELAS_ROBOT_ACTION_CONTROL_CLIENT_H


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/action_definition.h>


#include <dji_osdk_ros/dji_vehicle_node.h>
#include <sensyn_dji_gimbal_action/GimbalCameraAction.h>

#include <action_type_definition.h>

using namespace actionlib;
using namespace sensyn_dji_gimbal_action;
using namespace std;

namespace sr {

    class GimbalCameraActionClient : public SimpleActionClient<sensyn_dji_gimbal_action::GimbalCameraAction> {
        using SimpleActionClient::SimpleActionClient;

    public:
        void doneCb(const actionlib::SimpleClientGoalState &state, GimbalCameraResult &result);

        bool requestTaskAimCamera(const double roll, const double pitch, const double yaw, const double zoom, const double time) ;

        bool requestTaskTargetCamera(const double roll, const double x, const double y, const double z, const double zoom, const double time) ;

        bool requestTaskTargetCamera(const double roll, const double x, const double y, const double z, const double zoom, const double time, const bool is_reset) ;

        bool requestTaskTakePicture(const string camera_setting);

        bool requestTaskRecordVideo(const string camera_setting);

        bool requestTaskStartRecordVideo(const string camera_setting);

        bool requestTaskStopRecordVideo();



    };
}
// #endif //ELAS_ROBOT_ACTION_CONTROL_CLIENT_H
