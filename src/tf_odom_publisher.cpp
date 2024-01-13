#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>

namespace tf_odom_publisher {
  class tf_odom_publisher
  {
  public:
    tf_odom_publisher()
      : tfListener_(tfBuffer_)
    {
      ros::NodeHandle pnh("~");

      pnh.param("parent_frame_id", parent_frame_id_,
                std::string(""));
      pnh.param("child_frame_id", child_frame_id_,
                std::string(""));
      double rate_;
      pnh.param("publish_rate", rate_,
		500.0);

      odom_pub = pnh.advertise<nav_msgs::Odometry>("output", 10);
      timer_ = pnh.createTimer(ros::Duration(1.0 /rate_), &tf_odom_publisher::timerCallback, this);
    }

    void timerCallback(const ros::TimerEvent& event) {
      geometry_msgs::TransformStamped transform;
      try{
	transform = tfBuffer_.lookupTransform(parent_frame_id_, child_frame_id_, ros::Time(0), ros::Duration(3));
      } catch (std::exception& ex) {
        ROS_ERROR_STREAM(ex.what());
        return;
      }
      nav_msgs::Odometry odom_msg;

      odom_msg.header.stamp = transform.header.stamp;
      odom_msg.header.frame_id = transform.header.frame_id;
      odom_msg.child_frame_id = transform.child_frame_id;
      odom_msg.pose.pose.position.x = transform.transform.translation.x;
      odom_msg.pose.pose.position.y = transform.transform.translation.y;
      odom_msg.pose.pose.position.z = transform.transform.translation.z;
      odom_msg.pose.pose.orientation = transform.transform.rotation;
      odom_pub.publish(odom_msg);
    }

  protected:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    ros::Publisher odom_pub;
    ros::Timer timer_;
    std::string parent_frame_id_;
    std::string child_frame_id_;
  private:
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_odom_publisher");
  tf_odom_publisher::tf_odom_publisher c;
  ros::spin();
}
