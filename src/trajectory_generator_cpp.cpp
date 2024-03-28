#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h> // ロボットを動かすために必要
#include<geometry_msgs/Twist.h>
#include<math.h>

//ちょっと複雑になってきて厳しいのでPythonで書くわ
class Trajectory_generator{
	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber sub_vel;
		ros::Subscriber sub_stat;
	public:
		Trajectory_generator();
		void Callback_vel(const std_msgs::Float32MultiArray::ConstPtr& msg);
		void Callback_stat(const std_msgs::Bool::ConstPtr& msg);
		XmlRpc::XmlRpcValue walk_pattern_origin;
		ros::param::get("walk_pattern",walk_pattern_origin);
		const XmlRpc::XmlRpcValue& walk_pattern = walk_pattern_origin;
		bool robot_static;
		int lr;
		//前進・左折のパターンが入ってる　左足から動かすパターンとなっている
		//一度しか読まないので、破壊的変更をしないようにconst参照にしている
};

Trajectory_generator::Trajectory_generator()
{
	pub = nh.advertise<std_msgs::Float32MultiArray>("/foot_trajectory", 1);
	sub_vel = nh.subscribe("/cmd_vel_robot", 1, &Trajectory_generator::Callback_vel, this);
}

void Trajectory_generator::array_modifier(float time_rate, bool time_reverse, bool lr_reverse){
	std_msgs::Float32MultiArray pubarray;
	pubarray.data.resize()
}

void Trajectory_generator::Callback_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
	ros::param::get("/lr",lr)
	//lrは、最後に右を出した（次は左から出す）なら0、その逆なら1
	if(robot_static){
		//robot_static=trueならば次のtrajectoryをpubできる
		if (fabs(msg.linear.x) > fabs(msg.angular.z)*0.036){
			//前進・後退に丸めている 0.036はロボットの半径に相当 
		}else{
			//右回転・左回転
		}
	}
}

void Trajectory_generator::Callback_stat(const std_msgs::Bool::ConstPtr& msg){
	if(msg==nullptr){
		robot_static = false;
	}else{
		robot_static = msg;
	}
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "Trajectory_generator_cpp");
	
	Trajectory_generator Trajectory_generator;

	ros::Rate loop_rate(10);
	ros::spin();
	return 0;
}

