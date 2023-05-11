#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <dji_osdk_ros/dji_vehicle_node.h>
#include <actionlib/server/simple_action_server.h>
#include <sensyn_dji_gimbal_action/ControlGimbalAction.h>




class GimbalActionServer
{
public:
    
  GimbalActionServer(std::string name) : 
    as_controlGimbal(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks

    as_controlGimbal.registerGoalCallback(boost::bind(&GimbalActionServer::goalCB_controlGimbal, this));
    as_controlGimbal.registerPreemptCallback(boost::bind(&GimbalActionServer::preemptCB_controlGimbal, this));
    as_controlGimbal.start();
    
  }

  ~GimbalActionServer(void)
  {
  }



    void goalCB_controlGimbal()
  {
    sensyn_dji_gimbal_action::ControlGimbalGoal goal = *as_controlGimbal.acceptNewGoal();
    gimbal_action.request.is_reset = goal.is_reset;
    gimbal_action.request.pitch = goal.pitch;
    gimbal_action.request.roll = goal.roll;
    gimbal_action.request.yaw = goal.yaw;
    gimbal_action.request.time = goal.time;
    gimbal_action.request.payload_index = 0;
    gimbal_action.request.rotationMode = 0;

    sensyn_dji_gimbal_action::ControlGimbalResult result_controlGimbal;
    if (gimbal_action_client_.call(gimbal_action)) {
        ROS_INFO_STREAM(" gimbal_action success!");

        result_controlGimbal.done = true;
    } else {
        ROS_ERROR_STREAM(" gimbal_action failed!");
        result_controlGimbal.done = false;
    }
    as_controlGimbal.setSucceeded(result_controlGimbal);
  }

  void preemptCB_controlGimbal()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_controlGimbal.setPreempted();
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
  attitude_yaw = yaw;
  ROS_INFO("%f: attitude_yaw", attitude_yaw);
  return;
}
  

protected:
    
  ros::NodeHandle nh_;
  
  // actionlib::SimpleActionServer<sensyn_dji_gimbal_action::EnableTunnelModeAction> as_enableTunnelMode;
  actionlib::SimpleActionServer<sensyn_dji_gimbal_action::ControlGimbalAction> as_controlGimbal;

  ros::ServiceClient gimbal_action_client_ = nh_.serviceClient<dji_osdk_ros::GimbalAction>("/gimbal_task_control");
  dji_osdk_ros::GimbalAction gimbal_action;

  std::string action_name_;

  double attitude_yaw;

  ros::Subscriber sub_attitude = nh_.subscribe("/dji_osdk_ros/attitude", 1, &GimbalActionServer::attitude_callback, this);

};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "dji_gimbal_action_server");

  GimbalActionServer server(ros::this_node::getName());
  ros::spin();

  return 0;
}