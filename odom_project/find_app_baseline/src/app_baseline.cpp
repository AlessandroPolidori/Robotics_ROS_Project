#include "ros/ros.h"
#include "find_app_baseline/MotorSpeed.h"
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <math.h>


class BaseFinder
{

public:

	BaseFinder(): sub1(n, "motor_speed_rl", 1),sub2(n, "motor_speed_rr", 1),sub3(n, "motor_speed_fl", 1),
				  sub4(n, "motor_speed_fr", 1), sub5(n, "scout_odom", 1),sync5(sub1,sub2,sub3,sub4,sub5,10){      

	
	sub1.subscribe(n, "motor_speed_rl", 1);
	sub2.subscribe(n, "motor_speed_rr", 1);
	sub3.subscribe(n, "motor_speed_fl", 1);
	sub4.subscribe(n, "motor_speed_fr", 1);
	sub5.subscribe(n, "scout_odom", 1);

	sync5.registerCallback(boost::bind(&BaseFinder::callback,this, _1, _2,_3,_4,_5));

	}

protected:

    ros::NodeHandle n;

	float rpm_r;
	float rpm_l;
	float vr;
	float vl;
	float app_baseline;
	float w;
    float vx;
    float vy;
	float v_real;
    

    message_filters::Subscriber<find_app_baseline::MotorSpeed> sub1;
	message_filters::Subscriber<find_app_baseline::MotorSpeed> sub2;
	message_filters::Subscriber<find_app_baseline::MotorSpeed> sub3;
	message_filters::Subscriber<find_app_baseline::MotorSpeed> sub4;
	message_filters::Subscriber<nav_msgs::Odometry> sub5;

    message_filters::TimeSynchronizer<find_app_baseline::MotorSpeed,
					 				find_app_baseline::MotorSpeed,
					 				find_app_baseline::MotorSpeed,
									find_app_baseline::MotorSpeed,
                             		  			nav_msgs::Odometry> sync5;

public:

	void callback(const find_app_baseline::MotorSpeed::ConstPtr& rl, const find_app_baseline::MotorSpeed::ConstPtr& rr,
              	  const find_app_baseline::MotorSpeed::ConstPtr& fl, const find_app_baseline::MotorSpeed::ConstPtr& fr,
			  	  const nav_msgs::Odometry::ConstPtr& robotOdom){

		w = robotOdom->twist.twist.angular.z;
    	vx = robotOdom->twist.twist.linear.x;
    	vy = robotOdom->twist.twist.linear.y;
 
		v_real = sqrt((vx*vx) + (vy*vy)); //robot internal odometry's linear velocity
			
		ROS_INFO("vx: %f", vx);
		ROS_INFO("vy: %f", vy);
		ROS_INFO("v_real: %f", v_real); 
	
		rpm_r = (fr->rpm + rr->rpm)/(2*38.5); // 38.5 is the transmission ratio i found experimentally
		rpm_l = (fl->rpm + rl->rpm)/(2*38.5);
		vr = (rpm_r * (2 * 3.1416) * 1.575)/600; //right wheel linear velocity
		vl = -(rpm_l * (2 * 3.1416) * 1.575)/600; // left wheel linear velocity
		//ROS_INFO("vr: %f",vr);
		//ROS_INFO("vl: %f",vl);
		ROS_INFO("v_found: %f",( (vl+vr)/2 )  ); //printing baselink linear velocity
		
		if(w> 0.15 || w< -0.15){              //avoiding dividing by zero
			app_baseline = (vr-vl)/w;
			ROS_INFO("apparent baseline: %f",app_baseline); //seems like it's around 1.02
		
		}	

	}
	
};



       
int main(int argc, char **argv){
  	
	ros::init(argc, argv, "baseline_finder");


  	BaseFinder bf;
	
  	ros::spin();

  return 0;
}
