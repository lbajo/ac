#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include <csuro_tools/PIDController.h>

//filtro -> 33,5,255,0,255,0
//robot -> H0 h0 S255 s66 V255 V34

class FollowBall{

public:
	//string name, float minRef, float maxRef, float minOutput, float maxOutput 

	FollowBall(): Lin_("Lineal" , 0.1 , 1.0 , 0.1 , 1.0), Ang_("Angular" , 0.1 , 1.0 , 0.2 , 1.2)
	{
		pub_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	}

	geometry_msgs::Twist vel;

	void lostBall(){
		ROS_ERROR("PERDIDO");
		vel.linear.x=0.0;
		vel.angular.z=0.2;
	}

	void move(){

		try{
			listener.lookupTransform("/base_footprint", "/object/0", ros::Time(0) , bf2obj);
			ROS_INFO("/base_footprint -> /object/0 X[%lf] Y[ %lf] Z[%lf] %lf ago", bf2obj.getOrigin().x(), bf2obj.getOrigin().y(), bf2obj.getOrigin().z(), (ros::Time::now()-bf2obj.stamp_).toSec());
		}catch (tf::TransformException& ex){
			ROS_ERROR("%s",ex.what());
			lostBall();
		}

		float dist=0.0;
		float ang=0.0;
		dist=sqrt((bf2obj.getOrigin().x()*bf2obj.getOrigin().x()) + (bf2obj.getOrigin().y()*bf2obj.getOrigin().y()));
		ang= atan2(bf2obj.getOrigin().y(),bf2obj.getOrigin().x());

		if ((ros::Time::now()-bf2obj.stamp_).toSec() > 3.0){
			lostBall();
		}else{ 
			Lin_.setReference(dist - 0.5);
			Ang_.setReference(ang);
			vel.linear.x  = Lin_.getOutput();
			vel.angular.z = Ang_.getOutput();
		}

		ROS_INFO("Distancia [%f] Angulo [%f]",dist, ang);
		ROS_INFO("lin [%f], ang[%f]",vel.linear.x, vel.angular.z );

		pub_.publish(vel);
	}

private:
	tf::TransformListener listener;
	tf::StampedTransform bf2obj;
	ros::NodeHandle n_;
	ros::Publisher pub_;

	PIDController Lin_;
	PIDController Ang_;

	tf::StampedTransform last_tf_;
};


int main(int argc, char **argv){

	ros::init(argc, argv, "followball");

	FollowBall followball;

	ros::Rate loop_rate(20);

	while (ros::ok())
	{
		followball.move();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
