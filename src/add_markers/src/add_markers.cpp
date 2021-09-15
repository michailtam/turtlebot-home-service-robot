#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <typeinfo>

class OdomListener
{
  public:
    bool getFlg() { return bDestReached; }

    std::vector<double> getPosition() const { 
      if(position.size() != 0)
        return position;
      else { return std::vector<double> {0}; } // IMPORTANT: This is necessary because in first iteration the values are undefined 
    }
    std::vector<double> getOrientation() const { 
      if(orientation.size() != 0)
        return orientation;
      else { return std::vector<double> {0}; } // IMPORTANT: This is necessary because in first iteration the values are undefined 
    }
    std::vector<double> getAngularLinear() const {
      if(angular_linear.size() != 0)
        return angular_linear;
      else { return std::vector<double> {0}; } // IMPORTANT: This is necessary because in first iteration the values are undefined
    }

    void readOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      // ROS_INFO("Seq: [%d]", msg->header.seq);
      // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
      // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
      bDestReached = true;

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
      angular_linear = { 
        msg->twist.twist.linear.x, 
        msg->twist.twist.linear.y, 
        msg->twist.twist.linear.z 
       };
    }

  private:
    bool bDestReached = false;
    std::vector<double> position;
    std::vector<double> orientation;
    std::vector<double> angular_linear;
};


int main( int argc, char** argv )
{
  OdomListener odomListener;

  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1); // 1 Hz
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("odom", 1000, &OdomListener::readOdomCallback, &odomListener);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type to a CUBE
    marker.type = shape;

    // Set the marker action to ADD
    marker.action = visualization_msgs::Marker::ADD;

    // Set the position and orientation of the marker (6 DOF)
    marker.pose.position.x = 2.48455810546875;;
    marker.pose.position.y = -6.9661078453063965;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = -0.5930581109560585;
    marker.pose.orientation.w = 0.8051596593404511;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    ROS_INFO("Robot Pos: [%f, %f, %f] ", odomListener.getPosition()[0], odomListener.getPosition()[1], odomListener.getPosition()[2]);

    marker_pub.publish(marker); // Sets the marker

    // Wait 5 seconds and publish a DELETE message to remove the marker
    // ros::Duration(5).sleep();
    // marker.action = visualization_msgs::Marker::DELETE;
    // marker_pub.publish(marker);

    // Publish a ADD message to the marker at the drop off zone
    marker.action = visualization_msgs::Marker::ADD;
    
    // Set the position and orientation of the marker (6 DOF)
    marker.pose.position.x = -2.1428637504577637;
    marker.pose.position.y = -2.9643027782440186;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = -0.9198558849924803;
    marker.pose.orientation.w = 0.3922564860454202;

    marker_pub.publish(marker); // Sets the marker

    ros::spinOnce();
    r.sleep();
  }

  getchar();
}