#include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

class TwistConverter
{
	public:
		TwistConverter(ros::NodeHandle n) : n_(n), x_(0.0), y_(0.0), th_(0.0), received_first_message_(false) {}

		void run() {
			// geometry_msgs::TwistStampedConstPtr msg = ros::topic::waitForMessage<geometry_msgs::TwistStamped("/vehicle/status/twist", ros::Duration(100));

			subscriber_ = n_.subscribe("/vehicle/status/twist", 100, &TwistConverter::TwistCallback, this);
			ros::spin();
		}

		void TwistCallback(const geometry_msgs::TwistStamped& msg) {
			static tf2_ros::TransformBroadcaster br;
			if (received_first_message_) {
				//compute odometry in a typical way given the velocities of the robot
			    double dt = (msg.header.stamp - prev_t_).toSec();
			    double delta_x = (msg.twist.linear.x * cos(th_) - msg.twist.linear.y * sin(th_)) * dt;
			    double delta_y = (msg.twist.linear.x * sin(th_) + msg.twist.linear.y * cos(th_)) * dt;
			    double delta_th = msg.twist.angular.z * dt;

			    x_ += delta_x;
			    y_ += delta_y;
			    th_ += delta_th;

			    tf2::Quaternion odom_quat;
  				odom_quat.setRPY(0, 0, th_);
				geometry_msgs::TransformStamped odom_trans;
			    odom_trans.header.stamp = msg.header.stamp;
			    odom_trans.header.frame_id = "odom";
			    odom_trans.child_frame_id = "base_link";
			    odom_trans.transform.translation.x = x_;
			    odom_trans.transform.translation.y = y_;
			    odom_trans.transform.translation.z = 0.0;
			    odom_trans.transform.rotation.x = odom_quat.x();
				odom_trans.transform.rotation.y = odom_quat.y();
				odom_trans.transform.rotation.z = odom_quat.z();
				odom_trans.transform.rotation.w = odom_quat.w();

			    //send the transform
			    br.sendTransform(odom_trans);
			}

			// update previous time
			prev_t_ = msg.header.stamp;
			received_first_message_ = true;
		}
	private:
		ros::NodeHandle n_;
		ros::Subscriber subscriber_;

		ros::Time prev_t_;
		double x_;
		double y_;
		double th_;

		bool received_first_message_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "twist_converter");

  ros::NodeHandle n_;

  TwistConverter twist_converter(n_);
  twist_converter.run();

  return 0;
}