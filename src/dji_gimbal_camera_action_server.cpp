#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <dji_osdk_ros/dji_vehicle_node.h>
#include <actionlib/server/simple_action_server.h>
#include <sensyn_dji_gimbal_action/GimbalCameraAction.h>

#include <action_type_definition.h>

class GimbalCameraActionServer
{
public:
    
  GimbalCameraActionServer(std::string name) : 
    as_controlGimbalCamera(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks

    as_controlGimbalCamera.registerGoalCallback(boost::bind(&GimbalCameraActionServer::goalCB_controlGimbalCamera, this));
    as_controlGimbalCamera.registerPreemptCallback(boost::bind(&GimbalCameraActionServer::preemptCB_controlGimbalCamera, this));
    as_controlGimbalCamera.start();
    ROS_INFO("%s: as_controlGimbalCamera.start!!!", action_name_.c_str());
    
  }

  ~GimbalCameraActionServer(void)
  {
  }

    void goalCB_controlGimbalCamera()
  {
    sensyn_dji_gimbal_action::GimbalCameraGoal goal = *as_controlGimbalCamera.acceptNewGoal();

    sensyn_dji_gimbal_action::GimbalCameraResult result;
    result.done = doMission(goal);
    as_controlGimbalCamera.setSucceeded(result);
  }

  void preemptCB_controlGimbalCamera()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_controlGimbalCamera.setPreempted();
  }

  bool doMission(const sensyn_dji_gimbal_action::GimbalCameraGoal &goal) {

    uint8_t task_id = goal.task_id;

    GimbalCameraActionType action_type = GimbalCameraActionType(task_id);

    switch (action_type) {
        case GimbalCameraActionType::AimCamera:
            return ActionAimCamera(goal);
        case GimbalCameraActionType::TargetCamera:
            return ActionTargetCamera(goal);
        case GimbalCameraActionType::TakePicture:
            return ActionTakePicture(goal);
        case GimbalCameraActionType::RecordVideo:
            return ActionRecordVideo(goal);
        case GimbalCameraActionType::StartRecordVideo:
            return ActionStartRecordVideo(goal);
        case GimbalCameraActionType::StopRecordVideo:
            return ActionStopRecordVideo(goal);
        default:
            return false;
    }
}

bool ActionAimCamera(const sensyn_dji_gimbal_action::GimbalCameraGoal &goal){

  //first reset gimbal
  gimbal_action.request.is_reset = goal.is_reset;
  static double roll_offset = 0., pitch_offset = 0., yaw_offset = 0.;
  bool result;
  if (goal.is_reset) {
    
    if (gimbal_action_client_.call(gimbal_action)) {
        ROS_INFO_STREAM(" gimbal_action reset success!");
        roll_offset = gimbal_action.response.roll;
        pitch_offset = gimbal_action.response.pitch;
        yaw_offset = gimbal_action.response.yaw;

        result = true;
    } else {
        ROS_ERROR_STREAM(" gimbal_action reset failed!");
        result = false;
        return result;
    }
  }

  //rotate gimbal

  gimbal_action.request.is_reset = false;
  gimbal_action.request.pitch = goal.pitch + pitch_offset;
  gimbal_action.request.roll = goal.roll + roll_offset;
  gimbal_action.request.yaw = - goal.yaw + yaw_offset;
  // gimbal_action.request.yaw = - goal.yaw - attitude_yaw;
  gimbal_action.request.time = goal.time;
  gimbal_action.request.payload_index = 0;
  gimbal_action.request.rotationMode = 0;  

  if(gimbal_action.request.yaw>180){gimbal_action.request.yaw-=360;}
  if(gimbal_action.request.yaw<-180){gimbal_action.request.yaw+=360;}

    if (gimbal_action_client_.call(gimbal_action)) {
      ROS_INFO_STREAM(" gimbal_action success!");

      result = true;
  } else {
      ROS_ERROR_STREAM(" gimbal_action failed!");
      result = false;
      return result;
  }
  ros::Duration(goal.time).sleep();

  //zoom
  static double zoom_old = 1.;
  if (abs(zoom_old - goal.zoom) > 0.01) {

    zoom_old = goal.zoom;
    camera_task_set_zoom_para.request.factor = goal.zoom;

    if (zoom_client_.call(camera_task_set_zoom_para)) {
        ROS_INFO_STREAM(" zoom success!");

        result = true;
    } else {
        ROS_ERROR_STREAM(" zoom failed!");
        result = false;
    }
    ros::Duration(3.0).sleep();
  } 
  return result;

}

bool ActionTargetCamera(const sensyn_dji_gimbal_action::GimbalCameraGoal &goal){
  //coming soon...
  return true;
}

bool ActionTakePicture(const sensyn_dji_gimbal_action::GimbalCameraGoal &goal){

  //setting param is coming soon...

  camera_start_shoot_single_photo.request.payload_index=0;
  bool result;
  if (camera_single_action_client_.call(camera_start_shoot_single_photo)) {
      ROS_INFO_STREAM(" camera_single_shoot success!");

      result = true;
  } else {
      ROS_ERROR_STREAM(" camera_single_shoot failed!");
      result = false;
  }
  ros::Duration(1.0).sleep();
  return result;
}

bool ActionRecordVideo(const sensyn_dji_gimbal_action::GimbalCameraGoal &goal){
  //coming soon...
  return true;
}

bool ActionStartRecordVideo(const sensyn_dji_gimbal_action::GimbalCameraGoal &goal){

  //setting param is coming soon...

  camera_record_video_action.request.start_stop = 1;
  camera_record_video_action.request.payload_index=0;
  bool result;
  if (video_action_client_.call(camera_record_video_action)) {
      ROS_INFO_STREAM(" record success!");

      result = true;
  } else {
      ROS_ERROR_STREAM(" record failed!");
      result = false;
  }
  ros::Duration(1.0).sleep();
  return result;

}
bool ActionStopRecordVideo(const sensyn_dji_gimbal_action::GimbalCameraGoal &goal){
  camera_record_video_action.request.start_stop = 0;
  camera_record_video_action.request.payload_index=0;
  bool result;
  if (video_action_client_.call(camera_record_video_action)) {
      ROS_INFO_STREAM(" record success!");

      result = true;
  } else {
      ROS_ERROR_STREAM(" record failed!");
      result = false;
  }
  ros::Duration(1.0).sleep();
  return result;
}


void attitude_callback(geometry_msgs::QuaternionStamped attitude){
  tf::Quaternion q;
  q[0] = attitude.quaternion.x;
  q[1] = attitude.quaternion.y;
  q[2]= attitude.quaternion.z;
  q[3]= attitude.quaternion.w;
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  attitude_yaw = yaw*180/M_PI - 90;
  //ROS_INFO("%f: attitude_yaw", attitude_yaw);
  return;
}
  

protected:
    
  ros::NodeHandle nh_;
  
  // actionlib::SimpleActionServer<sensyn_dji_gimbal_action::EnableTunnelModeAction> as_enableTunnelMode;
  actionlib::SimpleActionServer<sensyn_dji_gimbal_action::GimbalCameraAction> as_controlGimbalCamera;

  ros::ServiceClient gimbal_action_client_ = nh_.serviceClient<dji_osdk_ros::GimbalAction>("/gimbal_task_control");
  dji_osdk_ros::GimbalAction gimbal_action;

  ros::ServiceClient video_action_client_ = nh_.serviceClient<dji_osdk_ros::CameraRecordVideoAction>("/camera_record_video_action");
  ros::ServiceClient camera_single_action_client_ = nh_.serviceClient<dji_osdk_ros::CameraStartShootSinglePhoto>("/camera_start_shoot_single_photo");
  ros::ServiceClient camera_interval_action_client_ = nh_.serviceClient<dji_osdk_ros::CameraStartShootIntervalPhoto>("/camera_start_shoot_interval_photo");
  ros::ServiceClient camera_stop_action_client_ = nh_.serviceClient<dji_osdk_ros::CameraStopShootPhoto>("/camera_stop_shoot_photo");

  dji_osdk_ros::CameraRecordVideoAction camera_record_video_action;
  dji_osdk_ros::CameraStartShootSinglePhoto camera_start_shoot_single_photo;
  dji_osdk_ros::CameraStartShootIntervalPhoto camera_start_shoot_interval_photo;
  dji_osdk_ros::CameraStopShootPhoto camera_stop_shoot_photo;

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

  double attitude_yaw;

  ros::Subscriber sub_attitude = nh_.subscribe("/dji_osdk_ros/attitude", 1, &GimbalCameraActionServer::attitude_callback, this);

};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "dji_gimbal_camera_action_server");

  GimbalCameraActionServer server(ros::this_node::getName());
  ros::spin();

  return 0;
}