#include "ros/ros.h"
#include <iostream>

#include "csuro_tools/Scan.h"

#include <tf/transform_broadcaster.h>//new
#include <tf/transform_listener.h>

#define PI 3.14159265

int main(int argc, char **argv)
{

   ros::init(argc, argv, "iterativo");
   ros::NodeHandle n;
   tf::StampedTransform object;
   tf::TransformBroadcaster br;

  tf::TransformListener listener;
  tf::StampedTransform odom2goal;

   ros::Rate loop_rate(20);

   Scan scan;

   int count=0;

   float dist_ant= 0.0;
   float ang_ant = 0.0;

   float dist_obs=0.0;
   float ang_obs =0.0;
   float array_objects[30];


   while (ros::ok())
   {
     ROS_INFO("iteration %d", count++);

     std::vector<tf::Stamped<tf::Point> > last_scan = scan.getLastScan();

     if(last_scan.size()>0 && (last_scan.begin()->stamp_ - ros::Time::now()).toSec() < 1.0)
     {
    	 std::vector<tf::Stamped<tf::Point> >::iterator it;

       float n_x=0.0;
       float n_y=0.0;
       int n=0;
       int num=0;

       float dists=0.0;
       float angs=0.0;
    	 for(it=last_scan.begin(); it!=last_scan.end(); ++it){
      //  if(it->getY()>0 && it->getX()<1.0 && it->getY()<1.0){
         
    		 //   std::cout<<"("<<it->getX()<<", "<<it->getY()<<", "<<it->getZ()<<") ";
          
       /*   n_x+=it->getX();
          n_y+=it->getY();
          n++;
        }*/

          if(it->getX()<1.1 && it->getX()>0 && it->getY()<1.5 && it->getY()>-1.5){
            dist_obs=0.0;
            ang_obs =0.0;
            //std::cout<<"("<<it->getX()<<", "<<it->getY()<<", "<<it->getZ()<<") ";
            ROS_INFO("X [%f] Y [%f]",it->getX(),it->getY());
            dist_obs=sqrt((it->getX()*it->getX()) + (it->getY()*it->getY()));
            ang_obs= atan2(it->getY(),it->getX());

            dists+=dist_obs;
            angs+=ang_obs;
            n++;

            dist_ant=dist_obs;
            ang_ant=ang_obs;

            //dist_obs=sqrt(pow(it->getX(),2) + pow(it->getY(),2));
            ROS_ERROR("DIST_OBS [%f] ANG_OBS [%f]",dist_obs, ang_obs);
          }

      }

      float media_dist=0.0;
      float media_ang=0.0;
      media_dist=dists/n;
      media_ang=angs/n;
      float ang_rep=media_ang+PI;

      ROS_ERROR("MEDIA_DIST [%f] MEDIA_ANG [%f] || ANG_REP [%f]",media_dist, media_ang, ang_rep);

      try{
        listener.lookupTransform("/odom", "/goal", ros::Time(0), odom2goal);
        ROS_INFO("/odom -> /goal [%lf, %lf, %lf] %lf ago", odom2goal.getOrigin().x(), odom2goal.getOrigin().y(), odom2goal.getOrigin().z(), (ros::Time::now()-odom2goal.stamp_).toSec());
        float dist_goal=sqrt((odom2goal.getOrigin().x()*odom2goal.getOrigin().x()) + (odom2goal.getOrigin().y()*odom2goal.getOrigin().y()));
        float ang_goal= atan2(odom2goal.getOrigin().y(),odom2goal.getOrigin().x());
        ROS_INFO("DIST GOAL [%f] ANG GOAL [%f]", dist_goal, ang_goal);
      }catch (tf::TransformException& ex){
        ROS_ERROR("%s",ex.what());
      }

      

 /*       float media_x = 0.0;
        float media_y = 0.0;
        media_y=n_y/n;
        media_x=n_x/n;  

        object.child_frame_id_ = "object";
        object.frame_id_ = "odom";
        object.stamp_ = ros::Time::now();


        object.setOrigin(tf::Vector3(-media_x,-media_y,0.0));
        //publico el vector de repulsión
        try {
          br.sendTransform(object);
        }catch(tf::TransformException &exception) {
          ROS_ERROR("%s", exception.what());
        }
*/
        //vector de repulsión-> -(media_x,media_y,0.0)
        // (odom2goal.getOrigin().x(),odom2goal.getOrigin().y(),0.0) menos el vector de repulsión
     }

     ros::spinOnce();
     loop_rate.sleep();
   }

   return 0;

 }
