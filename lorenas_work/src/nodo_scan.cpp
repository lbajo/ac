#include "ros/ros.h"
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include "csuro_tools/Scan.h"
#include <csuro_tools/PIDController.h>

typedef struct {
	float dist;
	float angle;
}Vector;

class Nodo_Scan{

public:
	Nodo_Scan();
	virtual ~Nodo_Scan();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
	const std::vector<tf::Stamped<tf::Point> >& getLastScan() const {return (scan_bf_);};
	bool obstacleDetected (float dist);
	void newObstacle(float dist, float angle);
	void initArray();
	void pubSubGoal();
	Vector calcVectorObst();
	void calcVectors(Vector vObs);
	void pubVel(float dist, float angle);

private:

	ros::NodeHandle nh_;
	ros::Publisher pub_;

	std::vector<tf::Stamped<tf::Point> > scan_bf_;
	std::string baseFrameId_;
	std::string laser_topic_;

	tf::TransformListener tfListener_;
	tf::TransformListener listener;

	tf::TransformBroadcaster br;

	tf::MessageFilter<sensor_msgs::LaserScan>* tfScanSub;
	message_filters::Subscriber<sensor_msgs::LaserScan>* scanSub;

	PIDController Ang_, Lin_;

	int numObs;

	Vector arrayObs[100];

	Vector vGoal;

	static const float ALPHA = 0.78;
	static const float BETHA = 0.3	;
	static const float MAXANG = 0.96;
	static const float MAXDIST = 1.5;
	static const float MINDIST = 0.75;
	static const float INFRONT = 0.2;
	static const float GOALAWAY = 0.4;
	static const float GOALCENTER = 0.2;

};

Nodo_Scan::Nodo_Scan():nh_(), baseFrameId_("base_footprint"), laser_topic_("/scan"), scan_bf_(), numObs(0), Lin_("Lineal",0.0 , 1.0 , 0.0, 0.2), Ang_("Angular" , 0.08 , 1, 0.0, 1.8)
{
	scanSub = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, laser_topic_, 5);
	tfScanSub = new tf::MessageFilter<sensor_msgs::LaserScan> (*scanSub, tfListener_, baseFrameId_, 5);
	tfScanSub -> registerCallback(boost::bind(&Nodo_Scan::scanCallback, this, _1));
	pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

	vGoal.dist = 0.0;
	vGoal.angle = 0.0;
}

Nodo_Scan::~Nodo_Scan()
{
		;
}

void
Nodo_Scan::pubSubGoal()
{
	tf::StampedTransform bf2odom;
	tf::StampedTransform odom2ball;
	tf::StampedTransform bf2goal2;
	tf::Transform bf2goal;

	try {
		listener.lookupTransform("/base_footprint", "/odom" , ros::Time(0), bf2odom);
		ROS_INFO("/base_footprint -> /goal [%lf, %lf, %lf] %lf ago", bf2odom.getOrigin().x(),
						bf2odom.getOrigin().y(), bf2odom.getOrigin().z(), (ros::Time::now() - bf2odom.stamp_).toSec());
	} catch (tf::TransformException& ex) {
		ROS_ERROR("%s", ex.what());
	}

	try {
		listener.lookupTransform("/odom", "/goal", ros::Time(0), odom2ball);
		ROS_INFO("/odom -> /goal [%lf, %lf, %lf] %lf ago", odom2ball.getOrigin().x(),
						odom2ball.getOrigin().y(), odom2ball.getOrigin().z(), (ros::Time::now() - odom2ball.stamp_).toSec());
	} catch (tf::TransformException& ex) {
			ROS_ERROR("%s", ex.what());
	}
	
	bf2goal = bf2odom * odom2ball;

	tf::Quaternion q;

	bf2goal2.setOrigin(tf::Vector3(bf2goal.getOrigin().x() , bf2goal.getOrigin().y() , bf2goal.getOrigin().z()));

	bf2goal2.child_frame_id_ = "bf2goal";
	bf2goal2.frame_id_ = "/base_footprint";
	bf2goal2.stamp_ = ros::Time::now();

	q.setRPY(0 , 0 , 0.1);
	bf2goal2.setRotation(q);

	try {
		br.sendTransform(bf2goal2);
	} catch(tf::TransformException &exception) {
		ROS_ERROR("%s", exception.what());
	}

	vGoal.dist = sqrt((bf2goal.getOrigin().x()*bf2goal.getOrigin().x()) + (bf2goal.getOrigin().y()*bf2goal.getOrigin().y()));
	vGoal.angle = atan2(bf2goal.getOrigin().y(), bf2goal.getOrigin().x());

}

void
Nodo_Scan::initArray()
{
	for (int i = 0; i < 100; i++){
		arrayObs[i].dist=0.0;
		arrayObs[i].angle=0.0;
	}
	numObs=0;
}

bool 
Nodo_Scan::obstacleDetected(float dist){
	if ((dist<2.0) && (dist>0.19))
		return true;
	return false;
}

void 
Nodo_Scan::newObstacle(float dist, float angle){
	arrayObs[numObs].dist=dist;
	arrayObs[numObs].angle=angle;
	numObs++;
}

Vector
Nodo_Scan::calcVectorObst(){

	Vector vObs;
	for (int i = 0; i < numObs; i++){
		if (abs(arrayObs[i].angle) <  MAXANG && arrayObs[i].dist <  MAXDIST){
			arrayObs[i].dist = MAXDIST - arrayObs[i].dist;
			arrayObs[i].angle =  MAXANG - arrayObs[i].angle; //dist

			vObs.dist+=arrayObs[i].dist;
			vObs.angle+=arrayObs[i].angle;
		}
	}

	return vObs;

}

void 
Nodo_Scan::pubVel(float dist, float angle){

	geometry_msgs::Twist vel;

	Lin_.setReference(dist);
	Ang_.setReference(angle);

	vel.linear.x = Lin_.getOutput();
	vel.angular.z = Ang_.getOutput();

	pub_.publish(vel);

}

void
Nodo_Scan::calcVectors(Vector vObs){
	Vector vGEnd;
	Vector vResult;

	vGEnd.dist = vGoal.dist * ALPHA;
	vGEnd.angle = vGoal.angle * ALPHA;

	// calculo el vector de atracción
		vGEnd.dist = vGoal.dist * ALPHA;
		vGEnd.angle =  vGoal.angle  * ALPHA;

		// Si me encuentro con algún obstáculo 
		vObs.dist = -vObs.dist * BETHA;
		vObs.angle  = -vObs.angle  * BETHA * 5;

		if (vObs.angle < INFRONT && vObs.angle > -INFRONT){
			// calcular vector del obstáculo
			ROS_ERROR("calcular vector del obstáculo");
			if (vObs.angle < INFRONT && vObs.angle > 0.0)
				vObs.angle =  0.3;
			else if (vObs.angle > -INFRONT && vObs.angle < 0.0)
				vObs.angle = -0.3;

			/* giro completo */
			ROS_ERROR("giro completo");
			vObs.dist = 0.0;
		}

		vResult.dist = vGEnd.dist + vObs.dist;
		vResult.angle = vGEnd.angle + vObs.angle;

		if (vResult.dist < 0){
			// no voy hacia atrás
			ROS_ERROR("no voy hacia atrás");
			vResult.dist = 0.1;
		}

		if (vResult.dist > -0.1 && vResult.dist < 0.1){
			ROS_ERROR("ir a goal");
			// ir a goal
			vResult.dist = 0.1;
		}
		if (vGEnd.dist < GOALAWAY && fabs(vGEnd.angle) < GOALCENTER){
			//  He llegado a goal
			ROS_ERROR("he llegado");
			vResult.dist = 0.0;
			vResult.angle = 0.0;
		}

		ROS_INFO("VECTOR GOAL : dist : %f angle : %f " , vGEnd.dist , vGEnd.angle);
		ROS_INFO("VECTOR OBSTACLE : dist : %f angle : %f " , vObs.dist , vObs.angle);
		ROS_ERROR("RESULT VECTOR : dist : %f angle : %f" , vResult.dist , vResult.angle);

		pubVel(vResult.angle , vResult.dist);
}

void
Nodo_Scan::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	float o_t_min, o_t_max, o_t_inc, dist, angle;

	o_t_min = scan_in->angle_min;
	o_t_max = scan_in->angle_max;
	o_t_inc = scan_in->angle_increment;

	int num_points = (int)2.0*o_t_max/o_t_inc;

	tf::Stamped<tf::Point> scan_sensor[num_points];
	scan_bf_.resize(num_points);

	float rx=0.0, ry=0.0;
	int c=0;

	initArray();

	for(int i=0; i<num_points; i++){
		float theta = o_t_min+i*o_t_inc;
		float r = scan_in->ranges[i];

		scan_sensor[i].setX(r*cos(theta));
		scan_sensor[i].setY(r*sin(theta));
		scan_sensor[i].setZ(0.0);
		scan_sensor[i].setW(1.0);
		scan_sensor[i].stamp_ = scan_in->header.stamp;
		scan_sensor[i].frame_id_ = scan_in->header.frame_id;

		tfListener_.transformPoint(baseFrameId_, scan_sensor[i], scan_bf_[i]);

		dist = sqrt((scan_bf_[i].getX()*scan_bf_[i].getX()) + (scan_bf_[i].getY()*scan_bf_[i].getY()));
        angle = atan2(scan_bf_[i].getY() , scan_bf_[i].getX());
        ROS_INFO("X [%f] Y [%f]",scan_bf_[i].getX(),scan_bf_[i].getY());
        if(obstacleDetected(dist))
        	newObstacle(dist,angle);
	}
	pubSubGoal();
	calcVectors(calcVectorObst());
}


int main(int argc, char **argv){

	ros::init(argc , argv , "Nodo_Scan");
	Nodo_Scan nodo_scan;
	ros::spin();
}

/*
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Actue_Node");
	Actue_Node actue_node;
	ros::Rate loop_rate(20);

	while (ros::ok()) {
		actue_node.transforms();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return (0);
}

*/