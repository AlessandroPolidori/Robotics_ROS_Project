#include "ros/ros.h"
#include <string.h>
#include <dynamic_reconfigure/server.h>
#include <skid_odometry/parametersConfig.h>
#include "skid_odometry/MotorSpeed.h"
#include "skid_odometry/odomCustom.h"
#include "skid_odometry/SetOdometry.h"
#include "skid_odometry/ResetOdometry.h"
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <math.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<skid_odometry::MotorSpeed,
					 									skid_odometry::MotorSpeed,
					 									skid_odometry::MotorSpeed,
														skid_odometry::MotorSpeed> SyncPolicy;


class Skid
{

public:

	/*Constructor*/
	Skid(): app_baseline{1.02}, x_pos{0.0}, y_pos{0.0},theta{0},runge_kutta{0},
			w{0.0},v_linear{0.0},v_x{0.0},v_y{0.0}, dt{0.0},
			sub1(n, "motor_speed_rl", 1), sub2(n, "motor_speed_rr", 1),			
			sub3(n, "motor_speed_fl", 1), sub4(n, "motor_speed_fr", 1), sync4(SyncPolicy(10),sub1,sub2,sub3,sub4){      


	/*if params are not set, all values are zeros*/
	n.getParam("starting_x_param",x_pos);
	n.getParam("starting_y_param",y_pos);
	n.getParam("starting_theta_param",theta);

	pub_odom = n.advertise<nav_msgs::Odometry>("/computed_odometry",10);
	pub_custom_odom = n.advertise<skid_odometry::odomCustom>("/computed_custom_odometry",10);

	sub1.subscribe(n, "motor_speed_rl", 1);
	sub2.subscribe(n, "motor_speed_rr", 1);
	sub3.subscribe(n, "motor_speed_fl", 1);
	sub4.subscribe(n, "motor_speed_fr", 1);
	
	/*registering message_filter*/
	sync4.registerCallback(boost::bind(&Skid::callback,this, _1, _2,_3,_4));
	/*registering services*/
	this->setService = n.advertiseService("set_odometry", &Skid::setOdometry,this);
	this->resetService = n.advertiseService("reset_odometry", &Skid::resetOdometry,this);

	}

private:


	float rpm_r;
	float rpm_l;
	float vr;
	float vl;
	float x_pos;
	float y_pos;
	float theta;
	float app_baseline;
	float v_linear;
	float v_x;
	float v_y;
	float w;
	
	bool runge_kutta;


	ros::Time time_ = ros::Time(0,0);
	float dt;

	ros::NodeHandle n;
	
	ros::Publisher pub_odom;
	ros::Publisher pub_custom_odom;


	ros::ServiceServer setService;
	ros::ServiceServer resetService;

	tf::TransformBroadcaster br;
	tf::Transform transform;

		

    message_filters::Subscriber<skid_odometry::MotorSpeed> sub1;
	message_filters::Subscriber<skid_odometry::MotorSpeed> sub2;
	message_filters::Subscriber<skid_odometry::MotorSpeed> sub3;
	message_filters::Subscriber<skid_odometry::MotorSpeed> sub4;
	

    message_filters::Synchronizer<SyncPolicy> sync4;

public:

	/*Message_filter callback*/
	void callback(const skid_odometry::MotorSpeed::ConstPtr& rl, const skid_odometry::MotorSpeed::ConstPtr& rr,
              	  const skid_odometry::MotorSpeed::ConstPtr& fl, const skid_odometry::MotorSpeed::ConstPtr& fr){

		
		
		this->rpm_r = (fr->rpm + rr->rpm)/(2*38.5); // 38.5 is the transmission ratio i found experimentally
		this->rpm_l = (fl->rpm + rl->rpm)/(2*38.5);
		this->vr = (rpm_r * (2 * 3.1416) * 1.575)/600; //right wheel linear velocity
		this->vl = -(rpm_l * (2 * 3.1416) * 1.575)/600; // left wheel linear velocity

		this->dt = (rl->header.stamp - time_).toSec(); // time between two calls
		this->time_ = rl->header.stamp; 

		calculate();

	}



	void calculate(){
		
		x_pos += v_x * dt;
		y_pos += v_y * dt;

		if (runge_kutta){

			v_x = v_linear * std::cos(theta + (w*dt / 2));
			v_y = v_linear * std::sin(theta + (w*dt / 2));

		}else{

			v_x = v_linear * std::cos(theta);
			v_y = v_linear * std::sin(theta);
			
		}

		theta += w * dt;
		

		v_linear = (vr + vl)/2;
		w = (vr - vl)/app_baseline;
        
		ROS_INFO("v_x: %f, v_y: %f, vlinear: %f",v_x,v_y,v_linear);
		publishOdomMessage();
		
		

	}

	void publishOdomMessage(){

		nav_msgs::Odometry odom;
    	odom.header.stamp = ros::Time::now();
    	odom.header.frame_id = "odom";
    	odom.child_frame_id = std::move("base_link");

    	odom.pose.pose.position.x = x_pos;
    	odom.pose.pose.position.y = y_pos;
    	odom.pose.pose.position.z = 0.0;
    	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    	odom.twist.twist.linear.x = v_x;
    	odom.twist.twist.linear.y = v_y;
    	odom.twist.twist.angular.z = w;

    	pub_odom.publish(odom);

		publishCustomOdomMessage(odom);
		broadCastTF();


	}

	
	void publishCustomOdomMessage(const nav_msgs::Odometry &odom){

		skid_odometry::odomCustom odom_custom;
		odom_custom.odom = odom;
		if(this->runge_kutta){odom_custom.method.data = std::move("rk");}
			else odom_custom.method.data = std::move("euler");
		pub_custom_odom.publish(odom_custom);
	
	}

	void broadCastTF(){
			
		transform.setOrigin( tf::Vector3(this->x_pos,this->y_pos,0));
		tf::Quaternion q;
		q.setRPY(0,0,this->theta);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, this->time_, "world","baseline"));

	}


	/*Services' callbacks*/
	bool setOdometry(skid_odometry::SetOdometry::Request  &req,
                     skid_odometry::SetOdometry::Response &res){
    	
		
		this->x_pos = req.x;
		this->y_pos = req.y;
		this->theta = req.theta;
		res.outcome = "Odometry has been set";
        return true;
	}

	bool resetOdometry(skid_odometry::ResetOdometry::Request  &req,
                     skid_odometry::ResetOdometry::Response &res){
    	
		ROS_INFO("entrato in reset odometry");
		this->x_pos = 0.0;
		this->y_pos = 0.0;
		this->theta = 0.0;
		res.outcome = "Odometry has been reset";
        return true;
	}
	

	/*Setters and Getters*/
	void setPositionX(float x){this->x_pos = x;}
	float getPositionX(){return this->x_pos;}
	void setPositionY(float y){this->y_pos = y;}
	void setIntegrationMethod(bool value){this->runge_kutta=value;}
	ros::NodeHandle getNodeHandler(){return this->n;}
    
		
};




void reconfigureOdometryParams(const skid_odometry::parametersConfig &params, uint32_t level, Skid &sk);



int main(int argc, char **argv){
  	
	ros::init(argc, argv, "skid_odometry_calculator");

	Skid sk;
	

	dynamic_reconfigure::Server<skid_odometry::parametersConfig> server;
	
	server.setCallback(boost::bind(&reconfigureOdometryParams, _1, _2, std::ref(sk)));

	
  	ros::spin();

  	return 0;
}


/*parameter dynamic reconfigure*/
void reconfigureOdometryParams(const skid_odometry::parametersConfig &params, uint32_t level, Skid &sk){

    sk.setIntegrationMethod(params.runge_kutta);
   
}
