#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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
    
    marker_pub.publish(marker); // Sets the marker

    // Wait 5 seconds and publish a DELETE message to remove the marker
    ros::Duration(5).sleep();
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    // Wait 5 seconds and publish a ADD message to the marker at the drop off zone
    ros::Duration(5).sleep();
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

    r.sleep();

    break;
  }
}