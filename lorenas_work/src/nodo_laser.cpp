#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"

#include <iostream>
#include "csuro_tools/Scan.h"

#define DIST_MAX	7.0
#define PI 3.14159265

class NodoLaser{

public:
	NodoLaser(): d(0.0), a(0.0)
	{
		pub_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	}

	geometry_msgs::Twist vel;
/*
	float calc(float atrac, float repuls){
		float angulo=0.0;
		float veloc=0.0;
		if(0<=atrac<=90 && 180<=repuls<=270){ //1º
			angulo=(atrac+repuls)/2;
		}else if(0<=atrac<=90 && 270<=repuls<=360){ //2º
			repuls=-(repuls-360);
			angulo=(atrac+repuls)/2;
		}else if(90<=atrac<=180 && 180<=repuls<=270){ //3º
			angulo=(atrac+repuls)/2;
		}else if(90<=atrac<=180 && 270<=repuls<=360){ //4º
			repuls=-(repuls-360);
			angulo=(atrac+repuls)/2;
		}

		if(0<=angulo<=90 ){ //1º
			veloc= -0.2;
		}else if(90<=angulo<=180 ){ //2º
			veloc= 0.2;
		}else if(180<=angulo<=270 ){ //3º
			veloc= 0.4;
		}else if(270<=angulo<=360 ){ //4º
			veloc= -0.4;
		}

	}
*/

	float angle(float angs, int n){
		float media_ang=0.0;
		media_ang=angs/n;
		//float ang_rep=media_ang+PI;
		return media_ang;
	}

	float distance(float dists, int n){
		float media_dist=0.0;
		media_dist=dists/n;
		return media_dist;

	}

	void data(){
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
				if(it->getX()<1.1 && it->getX()>0 && it->getY()<1.5 && it->getY()>-1.5){
	            	//std::cout<<"("<<it->getX()<<", "<<it->getY()<<", "<<it->getZ()<<") ";
	          //  	ROS_INFO("X [%f] Y [%f]",it->getX(),it->getY());
	            	dist_obs=sqrt((it->getX()*it->getX()) + (it->getY()*it->getY()));
	            	ang_obs= atan2(it->getY(),it->getX());

	            	dists+=dist_obs;
	            	angs+=ang_obs;
	            	n++;
	          
	            //dist_obs=sqrt(pow(it->getX(),2) + pow(it->getY(),2));
	            	//ROS_ERROR("DIST_OBS [%f] ANG_OBS [%f]",dist_obs, ang_obs);
	         	 }
	     	}
	     	ROS_ERROR("DIST_OBS [%f] ANG_OBS [%f]",dist_obs, ang_obs);
	     	angl=angle(angs,n);
	     	distan=distance(dists,n);
	    // 	a=angl;
	     //	d=distan;
	     	angle_rep=angle(dists,n)+PI;
	     	ROS_ERROR("MEDIA_DIST [%f] MEDIA_ANG [%f] || ANG_REP [%f]",distan, angl, angle_rep);
     	}
	}

	float calcAngl( tf::StampedTransform tf_)
	{
		return atan2(tf_.getOrigin().y(),tf_.getOrigin().x());
		
	}

	float calcDist( tf::StampedTransform tf_)
	{
		return sqrt(pow(tf_.getOrigin().x(),2) + pow(tf_.getOrigin().y(),2));

	}


	void move(){

		try{
			listener.lookupTransform("/base_footprint", "/goal", ros::Time(0), bf2goal);
			ROS_INFO("/base_footprint -> /goal [%lf, %lf, %lf] %lf ago", bf2goal.getOrigin().x(), bf2goal.getOrigin().y(), bf2goal.getOrigin().z(), (ros::Time::now()-bf2goal.stamp_).toSec());
		}
		catch (tf::TransformException& ex){
			ROS_ERROR("%s",ex.what());
		}

		d=calcDist(bf2goal);
		a=calcAngl(bf2goal);
		ROS_ERROR("calcDIST[%f], calcANG[%f]",d,a);
	/*	try{
			listener.lookupTransform("/base_footprint", "/odom", ros::Time(0), bf2odom);
			ROS_INFO("/base_footprint -> /goal [%lf, %lf, %lf] %lf ago", bf2goal.getOrigin().x(), bf2goal.getOrigin().y(), bf2goal.getOrigin().z(), (ros::Time::now()-bf2goal.stamp_).toSec());
		}
		catch (tf::TransformException& ex){
			ROS_ERROR("%s",ex.what());
		}*/
		float ang_goal=0.0;
		float dist_goal=0.0;
	/*	try{
			listener.lookupTransform("/odom", "/goal", ros::Time(0), odom2goal);
			ROS_INFO("/odom -> /goal [%lf, %lf, %lf] %lf ago", odom2goal.getOrigin().x(), odom2goal.getOrigin().y(), odom2goal.getOrigin().z(), (ros::Time::now()-odom2goal.stamp_).toSec());
			dist_goal=sqrt((odom2goal.getOrigin().x()*odom2goal.getOrigin().x()) + (odom2goal.getOrigin().y()*odom2goal.getOrigin().y()));
        	ang_goal= atan2(odom2goal.getOrigin().y(),odom2goal.getOrigin().x());
        	ROS_INFO("DIST GOAL [%f] ANG GOAL [%f]", dist_goal, ang_goal);

		}
		catch (tf::TransformException& ex){
			ROS_ERROR("%s",ex.what());
		}*/
/*

		float dist2goal=0.0;
		float ang2goal=0.0;
		float dist2rep=0.0;
		float ang2rep=0.0;

		dist2goal=sqrt((odom2goal.getOrigin().x()*odom2goal.getOrigin().x()) + (odom2goal.getOrigin().y()*odom2goal.getOrigin().y()));
		ang2goal= (atan2(odom2goal.getOrigin().y(),odom2goal.getOrigin().x()) * 180 )/ PI;
		dist2rep=sqrt((odom2goal.getOrigin().x()*odom2goal.getOrigin().x()) + (odom2goal.getOrigin().y()*odom2goal.getOrigin().y()));
		ang2rep= (atan2(odom2goal.getOrigin().y(),odom2goal.getOrigin().x()) * 180 )/ PI;
		ROS_INFO("Distancia [%f] Angulo [%f]",dist2goal, ang2goal);

		float vel_lin=0.0;
		float vel_ang=0.0;
		vel_lin=DIST_MAX/dist2goal;
		vel_ang=calc(ang2goal, ang2rep);

		vel.linear.x=vel_lin;
		vel.angular.z=vel_ang;

		ROS_INFO("Vel_lineal [%f] Vel_angular [%f]",vel_lin, vel_ang);
*/
	/*	if (dist_goal > 1.0)
			pub_.publish(vel);*/
	}

private:
	tf::TransformListener listener;
	tf::StampedTransform bf2odom;
	tf::StampedTransform bf2goal;
	tf::StampedTransform odom2goal;
	tf::StampedTransform odom2obj;
	ros::NodeHandle n_;
	ros::Publisher pub_;

	Scan scan;

	float d;
	float a;
};


int main(int argc, char **argv){

	ros::init(argc, argv, "nodolaser");

	NodoLaser nodolaser;

	ros::Rate loop_rate(20);

	while (ros::ok()){
		ROS_INFO("----------------------------------------------");
    	nodolaser.data();
    
		nodolaser.move();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
