#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>
#include <map>
#include <sstream>

enum class TargetZone { PICKUP_ZONE, DROP_OFF_ZONE };
enum class Operation { START, ADD_MARKER, DELETE_MARKER, MOVING, TASK_COMPLETED };

const std::vector<double> marker_prop { 0.1f, 0.1f, 0.2f, 0.0f, 1.0f, 1.0f, 1.0f }; // Size: 0-2, Color: 3-6

// The map contains the navigation data of each target point
const std::map<int, std::vector<double> > navData {
  { static_cast<int>(TargetZone::PICKUP_ZONE), std::vector<double> { 2.48f, -6.96f, 0.0f, 0.0f, 0.0f, -0.59f, 0.80f } }, // Position & orientation
  { static_cast<int>(TargetZone::DROP_OFF_ZONE), std::vector<double> { -2.14f, -2.96f, 0.0f, 0.0f, 0.0f, -0.92f, 0.39f } }
};

// Constains all the necessary task data
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

inline double calcManhattanDist(const std::vector<double>& robo_pose, const std::vector<double>& marker_pose) {
  return std::abs(marker_pose[0] - robo_pose[0]) + std::abs(marker_pose[1] - robo_pose[1]) + std::abs(marker_pose[2] - robo_pose[2]);
}

std::string enumtoString(const TargetZone& tarPos) {
    std::stringstream ss;
    switch(tarPos) {
    case TargetZone::PICKUP_ZONE: ss << "Pickup Zone"; break;
    case TargetZone::DROP_OFF_ZONE: ss << "Drop Off Zone"; break;
    default: ss << "Invalid target position!!!";
    }
    return ss.str();
}

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

void manageMarker(visualization_msgs::Marker& marker, TargetZone targetZone, Operation op, uint32_t shape) {
    if(op == Operation::ADD_MARKER) marker.action = visualization_msgs::Marker::ADD;
    else if (op == Operation::DELETE_MARKER) marker.action = visualization_msgs::Marker::DELETE;
    else ROS_INFO("No valid target position specified!!!"); 
      
    // Set the position and orientation of the markers pickup zone (6 DOF)
    marker.pose.position.x = navData.at(static_cast<int>(targetZone))[0];
    marker.pose.position.y = navData.at(static_cast<int>(targetZone))[1];
    marker.pose.position.z = navData.at(static_cast<int>(targetZone))[2];
    marker.pose.orientation.x = navData.at(static_cast<int>(targetZone))[3];
    marker.pose.orientation.y = navData.at(static_cast<int>(targetZone))[4];
    marker.pose.orientation.z = navData.at(static_cast<int>(targetZone))[5];
    marker.pose.orientation.w = navData.at(static_cast<int>(targetZone))[6];

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type to a CUBE
    marker.type = shape;
    
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = marker_prop[0];
    marker.scale.y = marker_prop[1];
    marker.scale.z = marker_prop[2];

    // Set the color -- be sure to set alpha to something non-zero!
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


int main( int argc, char** argv )
{
  AmclPoseListener amclPoseListener;
  ros::Subscriber sub;

  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n_pub_mark;  // Node handle for the marker publisher
  ros::NodeHandle n_sub_cmd("~"); // Node handle for the amcl_pose subscriber NOTE: ~ is necessary
  ros::NodeHandle n_sub_amcl;
  ros::Rate r(1); // 1 Hz
  
  // Read the parameters given by the command line
  std::string strParam;
  n_sub_cmd.getParam("operation", strParam);
  ROS_INFO("The parameter given is %s", strParam.c_str());

  ros::Publisher marker_pub = n_pub_mark.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  visualization_msgs::Marker marker { };
  double distance { };

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
    sub = n_sub_amcl.subscribe("amcl_pose", 1000, &AmclPoseListener::readPoseCallback, &amclPoseListener);
  }

  while (ros::ok())
  {
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return 0;
      }
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
    
    // // Executes the autonomous navigation task
    // if (strParam.compare("autonomous_nav") == 0) 
    // {
    //   // Check if the robot is moving towards the target zone
    //   if(operation == Operation::MOVING) {
    //     distance = calcManhattanDist(amclPoseListener.getPosition(), navData.at(targetID));
    //     ROS_INFO("\n[ROBOT IS MOVING] Distance to target %s is %f", enumtoString(targetZone).c_str(), distance);  
    //   } 
    //   // Add the marker to the pickup zone
    //   else if (operation == Operation::START) {
    //     ROS_INFO("\n[SHOW MARKER] at %s", enumtoString(targetZone).c_str());
    //     manageMarker(marker, static_cast<int>(targetZone), operation, shape);
    //     operation = Operation::MOVING;
    //   }
    //   // Check if the robot has reached the target zone
    //   else if(calcManhattanDist(navData.at(targetID), amclPoseListener.getPosition()) < 0.45f) 
    //   {
    //     // Determines which operation to execute based on the target zone
    //     switch(targetID)
    //     {
    //       case static_cast<int>(TargetZone::PICKUP_ZONE):
    //         // Show the marker to the pickup zone
    //         if(operation == Operation::ADD_MARKER) {
    //           ROS_INFO("[SHOW MARKER] at %s", enumtoString(targetZone).c_str());
    //           manageMarker(marker, static_cast<int>(targetZone), operation, shape);
    //           operation = Operation::DELETE_MARKER;  // (1) Next, hide the marker
    //         }
    //         // Hide the marker at the pickup zone
    //         else if(operation == Operation::DELETE_MARKER) {
    //           ROS_INFO("[HIDE MARKER] at %s", enumtoString(targetZone).c_str());
    //           operation = Operation::DELETE_MARKER;
    //           manageMarker(marker, static_cast<int>(targetZone), operation, shape);
    //           targetZone = TargetZone::DROP_OFF_ZONE;
    //           operation = Operation::MOVING;  // (2) Next, move the robot
    //         }
    //         break;
    //       case static_cast<int>(TargetZone::DROP_OFF_ZONE):
    //         if(operation == Operation::DELETE_MARKER) {
    //           ROS_INFO("[HIDE MARKER] at %s", enumtoString(targetZone).c_str());
    //           manageMarker(marker, static_cast<int>(targetZone), operation, shape);
    //         }
    //         break;
    //     }
    //   }
      
    marker_pub.publish(tskData.marker); // Publish the marker to the desired position (or hide it)

    ros::spinOnce();
    r.sleep();
  }

  getchar();
}