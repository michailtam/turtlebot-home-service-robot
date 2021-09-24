#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>
#include <map>
#include <sstream>

enum class TargetZone { PICKUP_ZONE, DROP_OFF_ZONE };
enum class Operation { START, ADD_MARKER, DELETE_MARKER, MOVING, TASK_COMPLETED };

// Properties of the marker
const std::vector<double> marker_prop { 0.1f, 0.1f, 0.2f, 0.0f, 1.0f, 1.0f, 1.0f }; // Size: 0-2, Color: 3-6

// The map contains the navigation data of each target point
const std::map<int, std::vector<double> > navData {
  { static_cast<int>(TargetZone::PICKUP_ZONE), std::vector<double> { 2.48f, -6.96f, 0.0f, 0.0f, 0.0f, -0.59f, 0.80f } }, // Position & orientation
  { static_cast<int>(TargetZone::DROP_OFF_ZONE), std::vector<double> { -2.14f, -2.96f, 0.0f, 0.0f, 0.0f, -0.92f, 0.39f } }
};

// Constains all the necessary data of the task
struct TaskData {
  TargetZone targetZone;
  Operation operation;
  visualization_msgs::Marker marker;
  uint32_t shape;

  TaskData(TargetZone tgZone, Operation op, visualization_msgs::Marker mk, uint32_t sh) {
    targetZone = tgZone;
    operation = op;
    marker = mk;
    shape = sh;
  }
};

/* Calculates the manhattan distance by the following formula: 
dist = |marker.x - robo.x| + |marker.y - robo.y| + |marker.z - robo.z| */
inline double calcManhattanDist(const std::vector<double>& robo_pose, const std::vector<double>& marker_pose) {
  return std::abs(marker_pose[0] - robo_pose[0]) + std::abs(marker_pose[1] - robo_pose[1]) + std::abs(marker_pose[2] - robo_pose[2]);
}

// Converts an enum value to a stl string
std::string enumtoString(const TargetZone& tarPos) {
    std::stringstream ss;
    switch(tarPos) {
    case TargetZone::PICKUP_ZONE: ss << "Pickup Zone"; break;
    case TargetZone::DROP_OFF_ZONE: ss << "Drop Off Zone"; break;
    default: ss << "Invalid target position!!!";
    }
    return ss.str();
}

// LISTENERS_FOR_SUBSCRIBER
class AmclPoseListener
{
  public:
    // IMPORTANT: Returning a zero vector is necessary because in first iteration the values are undefined 
    std::vector<double> getPosition() const { return (position.size() != 0) ? position : std::vector<double> {0}; }
    std::vector<double> getOrientation() const { return (orientation.size() != 0) ? orientation : std::vector<double> {0}; }

    void readPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
      position = { 
        msg->pose.pose.position.x, 
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
      };
      orientation = { 
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, 
        msg->pose.pose.orientation.z, 
        msg->pose.pose.orientation.w 
      };
    }

  private:
    std::vector<double> position;     // The position of the robot related to the map
    std::vector<double> orientation;  // The orientation of the robot related to the map
};

class CmdVelocityListener
{
  public:
    // IMPORTANT: Returning a zero vector is necessary because in first iteration the values are undefined 
    std::vector<double> getLinearVel() const { return (linear_vel.size() != 0) ? linear_vel : std::vector<double> {0}; }
    std::vector<double> getAngularVel() const { return (angular_vel.size() != 0) ? angular_vel : std::vector<double> {0}; }

    void readVelocitiesCallback(const geometry_msgs::Twist& msg) {
      linear_vel = { msg.linear.x, msg.linear.y, msg.linear.z };
      angular_vel = { msg.angular.x, msg.angular.y, msg.angular.z };
    }

  private:
    std::vector<double> linear_vel;   // The linear_velocity (x axis) of the robot
    std::vector<double> angular_vel;  // The angular_velocity (z axis) of the robot
};
// END LISTENERS_FOR_SUBSCRIBER

void manageMarker(visualization_msgs::Marker& marker, TargetZone targetZone, Operation op, uint32_t shape) {
    // Determine which action to perform
    if(op == Operation::ADD_MARKER) marker.action = visualization_msgs::Marker::ADD;
    else if (op == Operation::DELETE_MARKER) marker.action = visualization_msgs::Marker::DELETE;
    else ROS_INFO("Invalid target position specified!!!"); 
      
    // Set the position and orientation of the markers pickup zone (6 DOF)
    marker.pose.position.x = navData.at(static_cast<int>(targetZone))[0];
    marker.pose.position.y = navData.at(static_cast<int>(targetZone))[1];
    marker.pose.position.z = navData.at(static_cast<int>(targetZone))[2];
    marker.pose.orientation.x = navData.at(static_cast<int>(targetZone))[3];
    marker.pose.orientation.y = navData.at(static_cast<int>(targetZone))[4];
    marker.pose.orientation.z = navData.at(static_cast<int>(targetZone))[5];
    marker.pose.orientation.w = navData.at(static_cast<int>(targetZone))[6];

    // Set the frame ID, timestamp. See the TF tutorials for detailed information
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker. This serves to create a unique ID
    // NOTE: Any marker send with the same namespace and id, will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type
    marker.type = shape;
    
    // Set the scale of the marker (x,y,z)
    marker.scale.x = marker_prop[0];
    marker.scale.y = marker_prop[1];
    marker.scale.z = marker_prop[2];

    // Set the color, be sure to set alpha channel != 0 (R,G,B,A)
    marker.color.r = marker_prop[3];
    marker.color.g = marker_prop[4];
    marker.color.b = marker_prop[5];
    marker.color.a = marker_prop[6];
}

// ADD_MARKERS_OP
void add_markers_op(TaskData& tskData)
{
  switch(static_cast<int>(tskData.targetZone))
  {
    case static_cast<int>(TargetZone::PICKUP_ZONE):
      // Show the marker at the pickup zone
      if (tskData.operation == Operation::ADD_MARKER) {
        ROS_INFO("[SHOW MARKER] at %s", enumtoString(tskData.targetZone).c_str());
        manageMarker(tskData.marker, tskData.targetZone, tskData.operation, tskData.shape);
        tskData.operation = Operation::DELETE_MARKER;
      }
      // Hide the marker at the pickup zone
      else if(tskData.operation == Operation::DELETE_MARKER) {
        ros::Duration(5).sleep();
        ROS_INFO("[HIDE MARKER] at %s", enumtoString(tskData.targetZone).c_str());
        manageMarker(tskData.marker, tskData.targetZone, tskData.operation, tskData.shape);
        tskData.targetZone = TargetZone::DROP_OFF_ZONE;
        tskData.operation = Operation::ADD_MARKER;
      }
      break;
    case static_cast<int>(TargetZone::DROP_OFF_ZONE):
      // Show the marker to the drop off zone
      ros::Duration(5).sleep();
      ROS_INFO("[SHOW MARKER] at %s", enumtoString(tskData.targetZone).c_str());
      manageMarker(tskData.marker, tskData.targetZone, tskData.operation, tskData.shape);
      tskData.operation = Operation::TASK_COMPLETED;
      break;
    default:
      ROS_INFO("[WARNING] No target position defined!!!"); 
  }
}
// END ADD_MARKERS_OP

// Determines if the robot has stopped i.e. the angular velocity of all axis is zero
bool robotStopped(double angx, double angy, double angz) {
  if(angx == 0.0f && angy == 0.0f && angz == 0.0f) return true;
  else return false;
}

// AUTONOMOUS_NAVIGATION_OP
void autonomous_nav(TaskData& tskData, const AmclPoseListener* pAmclPoseListener, const CmdVelocityListener* pCmdVelocityListener)
{
  double distance = calcManhattanDist(pAmclPoseListener->getPosition(), navData.at(static_cast<int>(tskData.targetZone)));

  // Reaad the angular velocity
  double ang_x = pCmdVelocityListener->getAngularVel()[0];
  double ang_y = pCmdVelocityListener->getAngularVel()[1];
  double ang_z = pCmdVelocityListener->getAngularVel()[2];

  // Check if the robot is moving towards the target zone
  if(tskData.operation == Operation::MOVING) {
    // Checks if the robot has reached the target zone
    if(distance < 0.45f && robotStopped(ang_x, ang_y, ang_z) == true && tskData.operation == Operation::MOVING) {
      switch(static_cast<int>(tskData.targetZone)) {
        case static_cast<int>(TargetZone::PICKUP_ZONE):
          ROS_INFO("[OBJECT PICKED UP]");
          tskData.operation = Operation::DELETE_MARKER;
          break;
        case static_cast<int>(TargetZone::DROP_OFF_ZONE):
          ROS_INFO("[OBJECT DELIVERED]");
          tskData.operation = Operation::ADD_MARKER;
          break;
      }
    }
    // The robot moves towards the target zone
    else
      ROS_INFO("\n[ROBOT IS MOVING] Distance to target %s is %f", enumtoString(tskData.targetZone).c_str(), distance);  
  }
  // If the robot is not moving
  else 
  {
    switch(static_cast<int>(tskData.targetZone)) 
    {
      case static_cast<int>(TargetZone::PICKUP_ZONE):
        // Show the marker at the pickup zone
        if (tskData.operation == Operation::ADD_MARKER) {
          ROS_INFO("[SHOW MARKER] at %s", enumtoString(tskData.targetZone).c_str());
          manageMarker(tskData.marker, tskData.targetZone, tskData.operation, tskData.shape);
          tskData.operation = Operation::MOVING;
        }
        // Hide the marker at the pickup zone
        else if(tskData.operation == Operation::DELETE_MARKER) {
          ROS_INFO("[HIDE MARKER] at %s", enumtoString(tskData.targetZone).c_str());
          manageMarker(tskData.marker, tskData.targetZone, tskData.operation, tskData.shape);
          tskData.targetZone = TargetZone::DROP_OFF_ZONE;
          tskData.operation = Operation::MOVING;
        }
        break;
      case static_cast<int>(TargetZone::DROP_OFF_ZONE):
        // Show the marker to the drop off zone
        ROS_INFO("[SHOW MARKER] at %s", enumtoString(tskData.targetZone).c_str());
        manageMarker(tskData.marker, tskData.targetZone, tskData.operation, tskData.shape);
        tskData.operation = Operation::TASK_COMPLETED;
        break;
      default:
        ROS_INFO("[WARNING] No target position defined!!!"); 
    }
  }
}
// END AUTONOMOUS_NAVIGATION_OP


int main( int argc, char** argv )
{
  AmclPoseListener amclPoseListener;  // Defines the listener for the amcl_pose topic
  CmdVelocityListener cmdVelListener; // Defines the listener for the cmd_vel topic
  ros::Subscriber sub_amcl_pose;      // Defines the subscriber for the amcl_pose topic
  ros::Subscriber sub_cmd_vel;        // Defines the subscriber for the cmd_vel topic

  ros::init(argc, argv, "add_markers"); // Creates this node named as add_markers
  ros::NodeHandle n_pub_mark;     // Node handle for the marker publisher
  ros::NodeHandle n_sub_cmd("~"); // Node handle for the amcl_pose command line argument NOTE: ~ is necessary
  ros::NodeHandle n_sub_amcl;     // Node handle for the amcl_pose subscriber
  ros::NodeHandle n_sub_cmd_vel;  // Node handle for the cmd velocity subscriber
  ros::Rate r(1); // 1 Hz
  
  // Read the parameters given by the command line
  std::string strParam;
  n_sub_cmd.getParam("operation", strParam);
  ROS_INFO("The parameter given is %s", strParam.c_str());

  // Initialize the publisher for the marker
  ros::Publisher marker_pub = n_pub_mark.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  visualization_msgs::Marker marker { };

  // Contains all the data of the current task
  TaskData tskData { 
    TargetZone::PICKUP_ZONE,  // Set target zone to pickup zone
    Operation::START,         // Set the operation to start
    marker,                   // Contains the marker properties
    visualization_msgs::Marker::CUBE  // The shape to draw as a marker
  };

  // Checks which operation to execute
  if(strParam.compare("add_marker") == 0) {
    ROS_INFO("[ADD_MARKER OPERATION]");
  } 
  else if(strParam.compare("autonomous_nav") == 0) {
    ROS_INFO("[AUTONOMOUS NAVIGATION OPERATION]");
    // Initialize the publisher for the amcl_pose and cmd_vel topic
    sub_amcl_pose = n_sub_amcl.subscribe("amcl_pose", 1000, &AmclPoseListener::readPoseCallback, &amclPoseListener);
    sub_cmd_vel = n_sub_cmd_vel.subscribe("cmd_vel", 1000, &CmdVelocityListener::readVelocitiesCallback, &cmdVelListener);
  }

  while (ros::ok()) // loops as long as there is no driver error or the node does not shut down
  {
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok())
        return 0;
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    // Add the marker to the target position
    if(tskData.operation == Operation::START) {
      ROS_INFO("[BEGIN TASK]");
      tskData.operation = Operation::ADD_MARKER;
    }
    // Checks if the taks has completed. If it is, the node shuts down
    else if(tskData.operation == Operation::TASK_COMPLETED) {
      ros::Duration(5).sleep();
      ROS_INFO("[TASK COMPLETED] add_markers node shuts down now!!!");
      ros::shutdown();
    }

    // Check if the add_markers operation is desired
    if (strParam.compare("add_marker") == 0) {
      add_markers_op(tskData);
    }
    // Check if the autonoumous navigation operation is desired
    if (strParam.compare("autonomous_nav") == 0) {
      autonomous_nav(tskData, &amclPoseListener, &cmdVelListener);
    }  
      
    marker_pub.publish(tskData.marker); // Publish the marker to the desired position (or hide it)

    ros::spinOnce();
    r.sleep();
  }

  // getchar();
}