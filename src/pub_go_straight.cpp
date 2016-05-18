#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

class PubGoStraightVel
{
  private:
    ros::Publisher pub_go_straight_vel;
    ros::NodeHandle nh_;
    geometry_msgs::Twist go_straight_twist;
    tf::TransformListener listener_;

  public:
    PubGoStraightVel(ros::NodeHandle &nh){
      nh_ = nh;
      pub_go_straight_vel = nh_.advertise<geometry_msgs::Twist>("/icart_mini/cmd_vel", 1);
    }

    bool pubGoStraightVel(double distance)
    {
      listener_.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));

      tf::StampedTransform start_transform;
      tf::StampedTransform current_transform;

      listener_.lookupTransform("base_link", "odom", ros::Time(0), start_transform);

      go_straight_twist.linear.x = 0.4;
      go_straight_twist.linear.y = 0.0;
      go_straight_twist.linear.z = 0.0;

      ros::Rate r(10);
      bool done = false;
      while(!done && nh_.ok()){
        pub_go_straight_vel.publish(go_straight_twist);
        r.sleep();

        try
        {
          listener_.lookupTransform("base_link", "odom", ros::Time(0), current_transform);
        }catch(tf::TransformException ex){
          ROS_ERROR("%s", ex.what());
          break;
        }
        tf::Transform relative_transform = start_transform.inverse() * current_transform;
        double dist_moved = relative_transform.getOrigin().length();
        if(dist_moved > distance) done = true;
      }
      if (done) {
        go_straight_twist.linear.x = 0.0;
        go_straight_twist.linear.y = 0.0;
        go_straight_twist.linear.z = 0.0;

        pub_go_straight_vel.publish(go_straight_twist);
        return true;
      }
      return false;
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_go_straight_vel");
  ros::NodeHandle nh;

  PubGoStraightVel pubgostraightvel(nh);
  pubgostraightvel.pubGoStraightVel(1.0);
}
