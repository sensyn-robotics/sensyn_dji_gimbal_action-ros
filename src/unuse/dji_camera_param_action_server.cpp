#include <ros/ros.h>
#include <dji_osdk_ros/dji_vehicle_node.h>
#include <actionlib/server/simple_action_server.h>
#include <sensyn_dji_gimbal_action/CameraParamAction.h>

#include <dji_osdk_ros/CameraStartShootSinglePhoto.h>
#include <dji_osdk_ros/CameraStartShootIntervalPhoto.h>
#include <dji_osdk_ros/CameraStopShootPhoto.h>
#include <dji_osdk_ros/CameraRecordVideoAction.h>

class CameraParamActionServer
{
public:
    
  CameraParamActionServer(std::string name) : 
    as_cameraParam(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks

    as_cameraParam.registerGoalCallback(boost::bind(&CameraParamActionServer::goalCB_cameraParam, this));
    as_cameraParam.registerPreemptCallback(boost::bind(&CameraParamActionServer::preemptCB_cameraParam, this));
    as_cameraParam.start();
    
  }

  ~CameraParamActionServer(void)
  {
  }



    void goalCB_cameraParam()
  {
    sensyn_dji_gimbal_action::CameraParamGoal goal = *as_cameraParam.acceptNewGoal();
    sensyn_dji_gimbal_action::CameraParamResult result_cameraParam;

    if(goal.mode=="zoom"){
      camera_task_set_zoom_para.request.factor = goal.zoom_factor;
      camera_task_set_zoom_para.request.payload_index=0;

      if (zoom_client_.call(camera_task_set_zoom_para)) {
          ROS_INFO_STREAM(" zoom success!");

          result_cameraParam.done = true;
      } else {
          ROS_ERROR_STREAM(" zoom failed!");
          result_cameraParam.done = false;
      }
      ros::Duration(3.0).sleep();
      as_cameraParam.setSucceeded(result_cameraParam);

    }
    else if(goal.mode=="zoom_point"){
      camera_task_tap_zoom_point.request.multiplier=goal.zoom_point_multiplier;
      camera_task_tap_zoom_point.request.x=goal.zoom_point_x;
      camera_task_tap_zoom_point.request.y=goal.zoom_point_y;
      camera_task_tap_zoom_point.request.payload_index=0;

      if (zoom_point_client_.call(camera_task_tap_zoom_point)) {
          ROS_INFO_STREAM(" zoom_point success!");

          result_cameraParam.done = true;
      } else {
          ROS_ERROR_STREAM(" zoom_point failed!");
          result_cameraParam.done = false;
      }
      ros::Duration(3.0).sleep();
      as_cameraParam.setSucceeded(result_cameraParam);

    }
    else if(goal.mode=="focus"){
      camera_task_set_focus_point.request.x=goal.focus_x;
      camera_task_set_focus_point.request.y=goal.focus_y;
      camera_task_set_focus_point.request.payload_index=0;

      if (focus_client_.call(camera_task_set_focus_point)) {
          ROS_INFO_STREAM(" focus success!");

          result_cameraParam.done = true;
      } else {
          ROS_ERROR_STREAM(" focus failed!");
          result_cameraParam.done = false;
      }
      ros::Duration(1.0).sleep();
      as_cameraParam.setSucceeded(result_cameraParam);
    }
    else if(goal.mode=="EV"){
      camera_task_set_EV.request.exposure_mode=goal.exposure_mode;
      camera_task_set_EV.request.exposure_compensation=goal.exposure_compensation;
      camera_task_set_EV.request.payload_index      = 0;
      if (ev_client_.call(camera_task_set_EV)) {
          ROS_INFO_STREAM(" EV success!");

          result_cameraParam.done = true;
      } else {
          ROS_ERROR_STREAM(" EV failed!");
          result_cameraParam.done = false;
      }
      ros::Duration(1.0).sleep();
      as_cameraParam.setSucceeded(result_cameraParam);

    }
    else if(goal.mode=="aperture"){
      camera_task_set_aperture.request.exposure_mode=goal.exposure_mode;
      camera_task_set_aperture.request.aperture=goal.aperture;
      camera_task_set_aperture.request.payload_index      = 0;
      if (aperture_client_.call(camera_task_set_aperture)) {
          ROS_INFO_STREAM(" aperture success!");

          result_cameraParam.done = true;
      } else {
          ROS_ERROR_STREAM(" aperture failed!");
          result_cameraParam.done = false;
      }
      ros::Duration(1.0).sleep();
      as_cameraParam.setSucceeded(result_cameraParam);

    }
    else if(goal.mode=="ISO"){
      camera_task_set_ISO.request.exposure_mode=goal.exposure_mode;
      camera_task_set_ISO.request.iso_data=goal.iso_data;
      camera_task_set_ISO.request.payload_index      = 0;
      if (iso_client_.call(camera_task_set_ISO)) {
          ROS_INFO_STREAM(" ISO success!");

          result_cameraParam.done = true;
      } else {
          ROS_ERROR_STREAM(" ISO failed!");
          result_cameraParam.done = false;
      }
      ros::Duration(1.0).sleep();
      as_cameraParam.setSucceeded(result_cameraParam);

    }
    else if(goal.mode=="shutter_speed"){
      camera_task_set_shutter_speed.request.exposure_mode=goal.exposure_mode;
      camera_task_set_shutter_speed.request.shutter_speed=goal.shutter_speed;
      camera_task_set_shutter_speed.request.payload_index      = 0;
      if (shutter_speed_client_.call(camera_task_set_shutter_speed)) {
          ROS_INFO_STREAM(" shutter_speed success!");

          result_cameraParam.done = true;
      } else {
          ROS_ERROR_STREAM(" shutter_speed failed!");
          result_cameraParam.done = false;
      }
      ros::Duration(1.0).sleep();
      as_cameraParam.setSucceeded(result_cameraParam);

    }
    else{
        ROS_ERROR_STREAM(" camera_param_action failed! No such mode!");
    }

  }

  void preemptCB_cameraParam()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_cameraParam.setPreempted();
  }

protected:
    
  ros::NodeHandle nh_;
  
  // actionlib::SimpleActionServer<sensyn_dji_gimbal_action::EnableTunnelModeAction> as_enableTunnelMode;
  actionlib::SimpleActionServer<sensyn_dji_gimbal_action::CameraParamAction> as_cameraParam;

  ros::ServiceClient zoom_client_ = nh_.serviceClient<dji_osdk_ros::CameraSetZoomPara>("/camera_task_set_zoom_para");
  ros::ServiceClient zoom_point_client_ = nh_.serviceClient<dji_osdk_ros::CameraTapZoomPoint>("/camera_task_tap_zoom_point");
  ros::ServiceClient focus_client_ = nh_.serviceClient<dji_osdk_ros::CameraFocusPoint>("/camera_task_set_focus_point");
  ros::ServiceClient ev_client_ = nh_.serviceClient<dji_osdk_ros::CameraEV>("/camera_task_set_EV");
  ros::ServiceClient aperture_client_ = nh_.serviceClient<dji_osdk_ros::CameraAperture>("/camera_task_set_aperture");
  ros::ServiceClient iso_client_ = nh_.serviceClient<dji_osdk_ros::CameraISO>("/camera_task_set_ISO");
  ros::ServiceClient shutter_speed_client_ = nh_.serviceClient<dji_osdk_ros::CameraShutterSpeed>("/camera_task_set_shutter_speed");

  dji_osdk_ros::CameraSetZoomPara camera_task_set_zoom_para;
  dji_osdk_ros::CameraTapZoomPoint camera_task_tap_zoom_point;
  dji_osdk_ros::CameraFocusPoint camera_task_set_focus_point;
  dji_osdk_ros::CameraEV camera_task_set_EV;
  dji_osdk_ros::CameraAperture camera_task_set_aperture;
  dji_osdk_ros::CameraISO camera_task_set_ISO;
  dji_osdk_ros::CameraShutterSpeed camera_task_set_shutter_speed;


  std::string action_name_;

};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "dji_camera_param_action_server");

  CameraParamActionServer server(ros::this_node::getName());
  ros::spin();

  return 0;
}