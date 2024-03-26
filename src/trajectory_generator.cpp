#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // ロボットを動かすために必要

class Cmd_vel_relay{
	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber sub;
	public:
		Cmd_vel_relay();
		void Callback(const geometry_msgs::Twist::ConstPtr& msg);
};

Cmd_vel_relay::Cmd_vel_relay()
{
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_robot", 1);
	sub = nh.subscribe("/cmd_vel", 1, &Cmd_vel_relay::Callback, this);
}

void Cmd_vel_relay::Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	pub.publish(msg);
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "cmd_vel_relay");
	
	Cmd_vel_relay cmd_vel_relay;

	ros::Rate loop_rate(10);
	ros::spin();
	return 0;
}

