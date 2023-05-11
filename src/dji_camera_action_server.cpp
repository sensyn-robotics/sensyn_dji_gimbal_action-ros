#include <ros/ros.h>
#include <dji_osdk_ros/dji_vehicle_node.h>
#include <actionlib/server/simple_action_server.h>
#include <sensyn_dji_gimbal_action/ShootCameraAction.h>


class CameraActionServer
{
public:
    
  CameraActionServer(std::string name) : 
    as_shootCamera(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks

    as_shootCamera.registerGoalCallback(boost::bind(&CameraActionServer::goalCB_shootCamera, this));
    as_shootCamera.registerPreemptCallback(boost::bind(&CameraActionServer::preemptCB_shootCamera, this));
    as_shootCamera.start();
    
  }

  ~CameraActionServer(void)
  {
  }



    void goalCB_shootCamera()
  {
    sensyn_dji_gimbal_action::ShootCameraGoal goal = *as_shootCamera.acceptNewGoal();
    sensyn_dji_gimbal_action::ShootCameraResult result_shootCamera;

    if(goal.mode=="video"){
      camera_record_video_action.request.start_stop = goal.video_start_stop;
      camera_record_video_action.request.payload_index=0;

      if (video_action_client_.call(camera_record_video_action)) {
          ROS_INFO_STREAM(" record success!");

          result_shootCamera.done = true;
      } else {
          ROS_ERROR_STREAM(" record failed!");
          result_shootCamera.done = false;
      }
      as_shootCamera.setSucceeded(result_shootCamera);

    }
    else if(goal.mode=="camera_single_shoot"){
      camera_start_shoot_single_photo.request.payload_index=0;

      if (camera_single_action_client_.call(camera_start_shoot_single_photo)) {
          ROS_INFO_STREAM(" camera_single_shoot success!");

          result_shootCamera.done = true;
      } else {
          ROS_ERROR_STREAM(" camera_single_shoot failed!");
          result_shootCamera.done = false;
      }
      as_shootCamera.setSucceeded(result_shootCamera);

    }
    else if(goal.mode=="camera_interval_shoot"){
      camera_start_shoot_interval_photo.request.photo_num_conticap = goal.num_of_interval_shoot;
      camera_start_shoot_interval_photo.request.time_interval      = goal.time_interval_of_interval_shoot;
      camera_start_shoot_interval_photo.request.payload_index      = 0;

      if (camera_interval_action_client_.call(camera_start_shoot_interval_photo)) {
          ROS_INFO_STREAM(" camera_interval_shoot success!");

          result_shootCamera.done = true;
      } else {
          ROS_ERROR_STREAM(" camera_interval_shoot failed!");
          result_shootCamera.done = false;
      }
      as_shootCamera.setSucceeded(result_shootCamera);
    }
    else if(goal.mode=="camera_stop"){
      camera_stop_shoot_photo.request.payload_index      = 0;
      if (camera_stop_action_client_.call(camera_stop_shoot_photo)) {
          ROS_INFO_STREAM(" camera_stop success!");

          result_shootCamera.done = true;
      } else {
          ROS_ERROR_STREAM(" camera_stop failed!");
          result_shootCamera.done = false;
      }
      as_shootCamera.setSucceeded(result_shootCamera);

    }
    else{
        ROS_ERROR_STREAM(" camera_action failed! No such mode!");
    }

  }

  void preemptCB_shootCamera()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_shootCamera.setPreempted();
  }

protected:
    
  ros::NodeHandle nh_;
  
  // actionlib::SimpleActionServer<sensyn_dji_gimbal_action::EnableTunnelModeAction> as_enableTunnelMode;
  actionlib::SimpleActionServer<sensyn_dji_gimbal_action::ShootCameraAction> as_shootCamera;

  ros::ServiceClient video_action_client_ = nh_.serviceClient<dji_osdk_ros::CameraRecordVideoAction>("/camera_record_video_action");
  ros::ServiceClient camera_single_action_client_ = nh_.serviceClient<dji_osdk_ros::CameraStartShootSinglePhoto>("/camera_start_shoot_single_photo");
  ros::ServiceClient camera_interval_action_client_ = nh_.serviceClient<dji_osdk_ros::CameraStartShootIntervalPhoto>("/camera_start_shoot_interval_photo");
  ros::ServiceClient camera_stop_action_client_ = nh_.serviceClient<dji_osdk_ros::CameraStopShootPhoto>("/camera_stop_shoot_photo");

  dji_osdk_ros::CameraRecordVideoAction camera_record_video_action;
  dji_osdk_ros::CameraStartShootSinglePhoto camera_start_shoot_single_photo;
  dji_osdk_ros::CameraStartShootIntervalPhoto camera_start_shoot_interval_photo;
  dji_osdk_ros::CameraStopShootPhoto camera_stop_shoot_photo;

  std::string action_name_;

};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "dji_camera_action_server");

  CameraActionServer server(ros::this_node::getName());
  ros::spin();

  return 0;
}