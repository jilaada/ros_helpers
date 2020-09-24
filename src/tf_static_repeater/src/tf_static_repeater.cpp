#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"

#include <iostream>

class TFRepeater
{
 public: 
 	TFRepeater(ros::NodeHandle n) : n_(n) {}  

 	void run() {
 		publisher_ = n_.advertise<tf2_msgs::TFMessage>("/tf_static", 10000);

 		std::cout << "Waiting for message" << std::endl;
 		tf2_msgs::TFMessageConstPtr msg = ros::topic::waitForMessage<tf2_msgs::TFMessage>("/tf_static_ros1", ros::Duration(100));

 		if (msg != NULL) {
 			std::cout << "Finished initialization" << std::endl;
 			tf_msg_ = *msg;
 		} else {
 			std::cout << "Failed to receive message" << std::endl;
 			return;
 		}
		ros::Timer timer = n_.createTimer(ros::Duration(0.1), &TFRepeater::TimerCallback, this);
		std::cout << "Start publishing regular TF message @ 10Hz" << std::endl;
  		ros::spin();
 	}

	void TimerCallback(const ros::TimerEvent&) {
		ros::Time curr_time = ros::Time::now(); 
		for (auto it = tf_msg_.transforms.begin(); it != tf_msg_.transforms.end(); it++) {
			it->header.stamp = curr_time;
		}
		publisher_.publish(tf_msg_);
	}
 private:
 	ros::NodeHandle n_;
 	ros::Publisher publisher_;
 	ros::Subscriber subscriber_;
 	tf2_msgs::TFMessage tf_msg_;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tf_static_repeater");
	ros::NodeHandle n_;

	TFRepeater tf_repeater(n_);
	tf_repeater.run();

  	return 0;
}