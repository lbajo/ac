#include "ros/ros.h"
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include "csuro_tools/Scan.h"
#include <csuro_tools/PIDController.h>

#define DIST_MAX	7.0
#define PI 3.14159265

typedef struct {
	float dist;
	float angle;
}Vector;

class NodoDef{

public:
	NodoDef(): Lin_("Lineal" , 0.1 , 1.0 , 0.1 , 0.7), Ang_("Angular" , 0.1 , 0.0 , 0.2 , 0.6),LinR_("LinearR", 0.05, 0.8+1.0, 0.1, 0.6),AngR_("AngularR", 0.0, 0.0, 0.1, 0.6)
	{
		pub_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	}

	geometry_msgs::Twist vel;

	void vectAtrac(){
		vAtrac.dist = 0.0;
		vAtrac.angle = 0.0;
		float x = bf2goal.getOrigin().x();
		float y = bf2goal.getOrigin().y();
		vAtrac.dist = sqrt((x*x) + (y*y));
		vAtrac.angle = atan2(y,x);
		ROS_WARN("VECTOR DE ATRACCION ->>> dist [%f] ang [%f]",vAtrac.dist, vAtrac.angle);
	}

	void protoObs(float dist, float angle){
		int a, b, c;
		a=0;
		b=0;
		c=0;
		vel.linear.x= -0.03;
		pub_.publish(vel);		
		if (dist < 0.5){
			vel.linear.x = -0.00;
			a=1;
		}else if (angle > 0.0){
			vel.linear.x = 0.00;
			vel.angular.z = -0.2;
			b=1;

		}else if (angle < 0.0){
			vel.linear.x = 0.00;
			vel.angular.z = 0.2;
			c=1;
		}
		ROS_ERROR("REPULSION--> vel.x[%f] vel.z[%f]",vel.linear.x,vel.angular.z);
		pub_.publish(vel);

		if(a=1){

		}else if(b=1){
			vel.angular.z = 0.2;
		}else if(c=1){
			vel.angular.z = -0.2;
		}
		ROS_ERROR("REPULSION--> vel.x[%f] vel.z[%f]",vel.linear.x,vel.angular.z);
		pub_.publish(vel);
		vel.linear.x=0.02;
		pub_.publish(vel);

	}


	void tf(){

		try {
			listener.lookupTransform("/base_footprint", "/odom" , ros::Time(0), bf2odom);
			ROS_INFO("/base_footprint -> /odom [%lf, %lf, %lf] %lf ago", bf2odom.getOrigin().x(),
						bf2odom.getOrigin().y(), bf2odom.getOrigin().z(), (ros::Time::now() - bf2odom.stamp_).toSec());
		} catch (tf::TransformException& ex) {
			ROS_ERROR("%s", ex.what());
		}

		try {
			listener.lookupTransform("/odom", "/goal", ros::Time(0), odom2goal);
			ROS_INFO("/odom -> /goal [%lf, %lf, %lf] %lf ago", odom2goal.getOrigin().x(),
						odom2goal.getOrigin().y(), odom2goal.getOrigin().z(), (ros::Time::now() - odom2goal.stamp_).toSec());
		} catch (tf::TransformException& ex) {
			ROS_ERROR("%s", ex.what());
		}

		try {
			listener.lookupTransform("/goal", "/odom", ros::Time(0), goal2odom);
			ROS_INFO("/goal -> /odom [%lf, %lf, %lf] %lf ago", goal2odom.getOrigin().x(),
						goal2odom.getOrigin().y(), goal2odom.getOrigin().z(), (ros::Time::now() - goal2odom.stamp_).toSec());
		} catch (tf::TransformException& ex) {
			ROS_ERROR("%s", ex.what());
		}

		try {
			listener.lookupTransform("/base_footprint", "/goal", ros::Time(0), bf2goal);
			ROS_INFO("/base_footprint -> /goal [%lf, %lf, %lf] %lf ago", bf2goal.getOrigin().x(),
						bf2goal.getOrigin().y(), bf2goal.getOrigin().z(), (ros::Time::now() - bf2goal.stamp_).toSec());
		} catch (tf::TransformException& ex) {
			ROS_ERROR("%s", ex.what());
		}

		vectAtrac();
/*
		if(odom2goal.getOrigin().x() == 0.0){
			ROS_ERROR("FIIIIN");
			vel.linear.x  = 0.0;
			vel.angular.z = 0.0;
			pub_.publish(vel);
		}
*/
		//bf2goal = bf2odom * odom2goal;
	/*	ROS_INFO("/odom -> /goal [%lf, %lf, %lf] %lf ago", bf2goal.getOrigin().x(),
						bf2goal.getOrigin().y(), bf2goal.getOrigin().z(), (ros::Time::now() - bf2goal.stamp_).toSec());*/

		//ROS_WARN("DIST_OBS [%f] ANG_OBS [%f]",dist_obs, ang_obs);

	}

	void laserData(){
		vRepul.dist = 0.0;
	    vRepul.angle = 0.0;
		std::vector<tf::Stamped<tf::Point> > last_scan = scan.getLastScan();
		if(last_scan.size()>0 && (last_scan.begin()->stamp_ - ros::Time::now()).toSec() < 1.0){
			std::vector<tf::Stamped<tf::Point> >::iterator it;
			int n=0;
			float dists=0.0;
			float angs=0.0;
			float angl=0.0;
			float angle_rep=0.0;
			float distan=0.0;
			float dist_obs=0.0;
   			float ang_obs =0.0;
			for(it=last_scan.begin(); it!=last_scan.end(); ++it){
				if(it->getX()<0.9 && it->getX()>0 && it->getY()<1.4 && it->getY()>-1.4){
	            	//std::cout<<"("<<it->getX()<<", "<<it->getY()<<", "<<it->getZ()<<") ";
	          //  	ROS_INFO("X [%f] Y [%f]",it->getX(),it->getY());
	            	dist_obs = sqrt((it->getX()*it->getX()) + (it->getY()*it->getY()));
	            	ang_obs = atan2(it->getY(),it->getX());

	            	/*dists+=dist_obs;
	            	angs+=ang_obs;
	            	n++;*/
	          
	            //dist_obs=sqrt(pow(it->getX(),2) + pow(it->getY(),2));
	            	//ROS_ERROR("DIST_OBS [%f] ANG_OBS [%f]",dist_obs, ang_obs);
	         	 }
	     	}

	     	vRepul.dist = dist_obs;
	     	vRepul.angle = ang_obs ;
	     	ROS_ERROR("vR-->DIST_OBS [%f] ANG_OBS [%f]",dist_obs, ang_obs);
	    /* 	if(vRepul.dist!=0 || vRepul.angle !=0){
	     		ROS_WARN("VECTOR DE REPULSION ->>> dist [%f] ang [%f]",vRepul.dist, vRepul.angle);
	     		protoObs(vRepul.dist, vRepul.angle);

	     	}else{
	     		ROS_ERROR("TODO NORMAL");
	     	}*/

	     	
	     /*	angl=angle(angs,n);
	     	distan=distance(dists,n);
	    // 	a=angl;
	     //	d=distan;
	     	angle_rep=angle(dists,n)+PI;
	     	ROS_ERROR("MEDIA_DIST [%f] MEDIA_ANG [%f] || ANG_REP [%f]",distan, angl, angle_rep);*/
     	}
	}

	void todo(){
		if(vAtrac.dist <= 0.0){
			ROS_ERROR("FIIIIN");
			vel.linear.x  = 0.0;
			vel.angular.z = 0.0;
			pub_.publish(vel);
		}else{
			if(vRepul.dist!=0 || vRepul.angle !=0){
	     		ROS_WARN("VECTOR DE REPULSION ->>> dist [%f] ang [%f]",vRepul.dist, vRepul.angle);
	     	//	protoObs(vRepul.dist, vRepul.angle);
	     		LinR_.setReference(vRepul.dist);
				AngR_.setReference(vRepul.angle);
				//vel.linear.x = LinR_.getOutput();
				float lin=0.0;
				float ang=0.0;
				/*lin =LinR_.getOutput()/2;
				vel.linear.x = lin;*/
				ang= -AngR_.getOutput()/2;
				vel.linear.x =0.01;
				vel.angular.z = ang;
				ROS_ERROR("REPULSION--> vel.x[%f] vel.z[%f]",vel.linear.x,vel.angular.z);
			/*	if(vel.angular.z<0.0){
					vel.angular.z+=-0.15;
				}else if(vel.angular.z>0.0){
					vel.angular.z+=0.15;
				}
*/
	     	}else{
	     	/*	Lin_.setReference(vAtrac.dist);
				Ang_.setReference(vAtrac.angle);
				vel.linear.x = Lin_.getOutput();
				vel.angular.z = Ang_.getOutput();*/
				vel.linear.x = 0.07;
				vel.angular.z = 0.0;

				if(vAtrac.angle>=0.2){
					vel.linear.x = 0.02;
					vel.angular.z = 0.14;
				}else if(vAtrac.angle<=-0.2){
					vel.linear.x = 0.02;
					vel.angular.z = -0.14;

				}

				ROS_ERROR("TODO NORMAL--> vel.x[%f] vel.z[%f]",vel.linear.x,vel.angular.z);
				//ROS_ERROR("TODO NORMAL");
				//vel.linear.x  = 0.05;
				//pub_.publish(vel);
	    	}
	 	}
	 	pub_.publish(vel);
	}


private:
	tf::TransformListener listener;
	tf::StampedTransform bf2odom;
	tf::StampedTransform bf2goal;
	tf::StampedTransform odom2goal;
	tf::StampedTransform goal2odom;
	tf::StampedTransform odom2obj;
	ros::NodeHandle n_;
	ros::Publisher pub_;

	Scan scan;

	float d;
	float a;

	PIDController Lin_;
	PIDController Ang_;
	PIDController LinR_;
	PIDController AngR_;

	Vector vAtrac;
	Vector vRepul;
};


int main(int argc, char **argv){

	ros::init(argc, argv, "nododef");

	NodoDef nd;

	ros::Rate loop_rate(20);

	while (ros::ok()){
		ROS_INFO("-------------------------------------------------------------------------------");
    	nd.laserData();
    	nd.tf();
    	nd.todo();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
