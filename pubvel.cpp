//This program publishes randomly-generated velocity
//messages for turtlesim.
#include"ros/ros.h"
#include"geometry_msgs/Pose2D.h" //For geometry_msgs::Pose
#include<cstdlib>
#include<ctime>

int main(int argc, char** argv){
	//Initialize the ROS system and become a node.
	ros::init(argc, argv, "publish_velocity");
	ros::NodeHandle nh;

	//Create a publisher obj ect.
	ros::Publisher pub = nh.advertise<geometry_msgs::Pose2D>(
			"robot_pose", 1000);

	//Seed the random number generator.
	srand(time(0));

	//Loop at 2Hz until the node is shut down.
	ros::Rate rate(2);
	while (ros::ok()){
		//Create and fill in the message. The other four
		//fields. which are ignored by turtlesim, default to 0.
		geometry_msgs::Pose2D msg;
		msg.x = double(rand()) / double(RAND_MAX);
		msg.y = 2 * double(rand()) / double(RAND_MAX) - 1;

		//Publish the message.
		pub.publish(msg);

		//Send a message to rosout with the details.
		ROS_INFO_STREAM("Sending random velocity command."
				<< "X-speed=" << msg.x
				<< "Y-speed=" << msg.y);

		//Wait until it's time for another iteration.
		rate.sleep();
	}
}
