#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>
#include <map>
#include <sstream>

enum class TargetPos { PICKUP_ZONE, DROP_OFF_ZONE };

const std::vector<double> marker_prop { 0.1f, 0.1f, 0.2f, 0.0f, 1.0f, 1.0f, 1.0f }; // Size: 0-2, Color: 3-6

// The map contains the navigation data of each target point
const std::map<int, std::vector<double> > navData {
  { static_cast<int>(TargetPos::PICKUP_ZONE), std::vector<double> { 2.48f, -6.96f, 0.0f, 0.0f, 0.0f, -0.59f, 0.80f } }, // Position & orientation
  { static_cast<int>(TargetPos::DROP_OFF_ZONE), std::vector<double> { -2.14f, -2.96f, 0.0f, 0.0f, 0.0f, -0.92f, 0.39f } }
};

inline double calcManhattanDist(const std::vector<double>& robo_pose, const std::vector<double>& marker_pose) {
  return std::abs(marker_pose[0] - robo_pose[0]) + std::abs(marker_pose[1] - robo_pose[1]) + std::abs(marker_pose[2] - robo_pose[2]);
}

std::string enumtoString(const TargetPos& tarPos) {
    std::stringstream ss;
    switch(tarPos) {
    case TargetPos::PICKUP_ZONE: ss << "Pickup Zone"; break;
    case TargetPos::DROP_OFF_ZONE: ss << "Drop Off Zone"; break;
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

void manageMarker(visualization_msgs::Marker& marker, int navDataID, const std::string& op, uint32_t shape) {
    if(op.compare("ADD") == 0) marker.action = visualization_msgs::Marker::ADD;
    else if (op.compare("DELETE") == 0) marker.action = visualization_msgs::Marker::DELETE;
    else ROS_INFO("No valid target position specified!!!"); 
      
    // Set the position and orientation of the markers pickup zone (6 DOF)
    marker.pose.position.x = navData.at(navDataID)[0];
    marker.pose.position.y = navData.at(navDataID)[1];
    marker.pose.position.z = navData.at(navDataID)[2];
    marker.pose.orientation.x = navData.at(navDataID)[3];
    marker.pose.orientation.y = navData.at(navDataID)[4];
    marker.pose.orientation.z = navData.at(navDataID)[5];
    marker.pose.orientation.w = navData.at(navDataID)[6];

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

int main( int argc, char** argv )
{
  AmclPoseListener amclPoseListener;
  
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1); // 1 Hz
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("amcl_pose", 1000, &AmclPoseListener::readPoseCallback, &amclPoseListener);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  int targetID = static_cast<int>(TargetPos::PICKUP_ZONE);
  visualization_msgs::Marker marker;

  manageMarker(marker, 0, "ADD", shape);  // Add the marker to the pick up zone
  std::string strMarker { enumtoString(TargetPos::PICKUP_ZONE) };

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

    if(calcManhattanDist(navData.at(targetID), amclPoseListener.getPosition()) < 0.45f) {
      // Execute operation based on the target ID
      switch(targetID) {
        case static_cast<int>(TargetPos::PICKUP_ZONE):
          ros::Duration(1).sleep();
          manageMarker(marker, 0, "DELETE", shape); // Remove the marker to the pick up zone
          targetID = static_cast<int>(TargetPos::DROP_OFF_ZONE);
          break;
        case static_cast<int>(TargetPos::DROP_OFF_ZONE):
          ros::Duration(2).sleep();
          manageMarker(marker, 1, "ADD", shape); // Add the marker to the drop off zone
          break;
        default:
          ROS_INFO("[WARNING] No target position defined!!!");
      }
    } 
    else  {
      ROS_INFO("\n[MOVING] Distance to target %s is %f", strMarker.c_str(), 
        calcManhattanDist(amclPoseListener.getPosition(), navData.at(targetID)));
    }

    marker_pub.publish(marker);

    ros::spinOnce();
    r.sleep();
  }

  getchar();
}