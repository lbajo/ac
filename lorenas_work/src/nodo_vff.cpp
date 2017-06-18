#include "ros/ros.h"
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include "csuro_tools/Scan.h"
#include <csuro_tools/PIDController.h>

#define DIST_MAX 7.0

typedef struct {
	float ptoX;
	float ptoY;
}Vector;

class NodoVFF{
public:
	NodoVFF(): Lin_("Lineal" , 0.0 , 1.0 , 0.1 , 0.6), Ang_("Angular" , 0.00 , 0.0 , 0.1 , 0.4)
	{
		pub_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	}

	geometry_msgs::Twist vel;

	void vectAtrac(){
		float mod = 0.0;
		vAtrac.ptoX = 0.0;
		vAtrac.ptoY = 0.0;

		float x = bf2goal.getOrigin().x();
		float y = bf2goal.getOrigin().y();

		mod = sqrt((x*x) + (y*y));
		vAtrac.ptoX = x/mod;
		vAtrac.ptoY = y/mod;

		ROS_WARN("VECTOR DE ATRACCION ->>> X [%f] Y [%f]", vAtrac.ptoX, vAtrac.ptoY);
	}

	void vectRepuls(float x, float y){
		ROS_INFO("VECTOR DE REPULSION-INICIAL ->>> X [%f] Y [%f]", x, y);
		float xRep = 0.0;
		float yRep = 0.0;
		vRepul.ptoX = 0.0;
		vRepul.ptoY = 0.0;

		if (y >= 0){
			yRep = - (1 - y);
		}else{
			yRep = 1 + y;
		}
		xRep = - (1 - x);

		vRepul.ptoX = xRep;
		vRepul.ptoY = yRep;
		if (x==0 && y==0){
			vRepul.ptoX = 0;
			vRepul.ptoY = 0;
			ROS_ERROR("CASO 0");
		}

		ROS_WARN("VECTOR DE REPULSION ->>> X [%f] Y [%f]", vRepul.ptoX, vRepul.ptoY);
	}

	void vectResult(){
		vResult.ptoX = 0.0;
		vResult.ptoY = 0.0;
		vResult.ptoX = vAtrac.ptoX + vRepul.ptoX;
		vResult.ptoY = vAtrac.ptoY + vRepul.ptoY;

		ROS_WARN("VECTOR RESULTANTE ->>> X [%f] Y [%f]", vResult.ptoX, vResult.ptoY);
	}

	void tf(){

		try {
			listener.lookupTransform("/odom", "/goal", ros::Time(0), odom2goal);
			ROS_INFO("/odom -> /goal [%lf, %lf, %lf] %lf ago", odom2goal.getOrigin().x(),
						odom2goal.getOrigin().y(), odom2goal.getOrigin().z(), (ros::Time::now() - odom2goal.stamp_).toSec());
		} catch (tf::TransformException& ex) {
			ROS_ERROR("%s", ex.what());
		}

		try {
			listener.lookupTransform("/base_footprint", "/goal" , ros::Time(0), bf2goal);
			ROS_INFO("/base_footprint -> /odom [%lf, %lf, %lf] %lf ago", bf2goal.getOrigin().x(),
						bf2goal.getOrigin().y(), bf2goal.getOrigin().z(), (ros::Time::now() - bf2goal.stamp_).toSec());
		} catch (tf::TransformException& ex) {
			ROS_ERROR("%s", ex.what());
		}

		vectAtrac();

	}

	void laserData(){
		std::vector<tf::Stamped<tf::Point> > last_scan = scan.getLastScan();
		if(last_scan.size()>0 && (last_scan.begin()->stamp_ - ros::Time::now()).toSec() < 1.0){
			std::vector<tf::Stamped<tf::Point> >::iterator it;

			int n = 0;

   			float x = 0.0;
   			float y = 0.0;
   			float xs =0.0;
   			float ys = 0.0;
   			float media_x = 0.0;
   			float media_y = 0.0;

			for(it=last_scan.begin(); it!=last_scan.end(); ++it){
				if(it->getX()<1.0 && it->getX()>0 && it->getY()<1.5 && it->getY()>-1.5){
	            	//std::cout<<"("<<it->getX()<<", "<<it->getY()<<", "<<it->getZ()<<") ";

	            	x = it->getX();
	            	y = it->getY();

	            	xs += x;
	            	ys += y;

	            	n++;
	         	 }
	     	}

	     	media_x = xs/n;
	     	media_y = ys/n;
	     	if (n < 2){
	     		media_x = 0;
	     		media_y = 0;
	     	}
	     	vectRepuls(media_x,media_y);
     	}
	}

	void move(){
		float dist = 0.0;
		float ang = 0.0;

		vectResult();

		dist = sqrt((vResult.ptoX * vResult.ptoX) + (vResult.ptoY * vResult.ptoY));
	    ang = atan2(vResult.ptoY,vResult.ptoY);

	    ROS_ERROR("DISTANCIA [%f] ANGULO [%f]", dist, ang);

	/*    Lin_.setReference(dist);
		Ang_.setReference(ang);

		vel.linear.x = Lin_.getOutput() - 0.1;
		vel.angular.z = Ang_.getOutput();*/

		vel.linear.x = 0.1;
		if (ang > 0.1){
			vel.angular.z = 0.2;
		}else if(ang > 0 && ang < 0.1){
			vel.angular.z = 0.1;
		}else if(ang < 0 && ang > -0.1){
			vel.angular.z = -0.1;
		}else if(ang < 0 && ang < -0.1){
			vel.angular.z = -0.2;
		}else{
			vel.angular.z = 0.0;
		}


		ROS_ERROR("VELOCIDADES vel.x[%f] vel.z[%f]",vel.linear.x,vel.angular.z);

		pub_.publish(vel);
	}


private:
	tf::TransformListener listener;
	tf::StampedTransform bf2goal;
	tf::StampedTransform odom2goal;

	ros::NodeHandle n_;
	ros::Publisher pub_;

	Scan scan;

	PIDController Lin_;
	PIDController Ang_;

	Vector vAtrac;
	Vector vRepul;
	Vector vResult;
};


int main(int argc, char **argv){

	ros::init(argc, argv, "nodovff");

	NodoVFF nVff;

	ros::Rate loop_rate(20);

	while (ros::ok()){
		ROS_INFO("-------------------------------------------------------------------------------");
		nVff.tf();
    	nVff.laserData();
    	nVff.move();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
