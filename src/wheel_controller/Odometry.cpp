
#include "Odometry.h"

Odometry_calc::Odometry_calc(bool reverse){
	init_variables();

	ROS_INFO("Started odometry computing node");

  	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);   
	mReverse = reverse;
  	
	//Retrieving parameters of this node
	get_node_params();
}

void Odometry_calc::init_variables()
{
	prev_lencoder = 0;
	prev_rencoder = 0;

	lmult = 0;
	rmult = 0;

	left = 0;
	right = 0;

	encoder_min =  -2147483648;
	encoder_max =  2147483648;

	rate = 10;

	ticks_meter = ENCODER_TICKS_PER_SINGLE_MOTOR_REVOLUTION / (WHEEL_DIAMETER_m * M_PI);

	base_width = WHEEL_DISTANCE_m;

	encoder_low_wrap = ((encoder_max - encoder_min) * 0.3) + encoder_min ;
	encoder_high_wrap = ((encoder_max - encoder_min) * 0.7) + encoder_min ;

	t_delta = ros::Duration(1.0 / rate);
	t_next = ros::Time::now() + t_delta;
	
	then = ros::Time::now();


	enc_left = 0;
	enc_right = 0;

	dx = 0;
	dr = 0;
 
	x_final = 0;y_final=0;theta_final=0;
	
	current_time = ros::Time::now();
  	last_time = ros::Time::now();

}
void Odometry_calc::get_node_params(){

	
        if(n.getParam("rate", rate)){
	 
		ROS_INFO_STREAM("Rate from param" << rate);	       
	}

        if(n.getParam("encoder_min", encoder_min)){
	 
		ROS_INFO_STREAM("Encoder min from param" << encoder_min);	       
	}


        if(n.getParam("encoder_max", encoder_max)){
	 
		ROS_INFO_STREAM("Encoder max from param" << encoder_max);	       
	}


        if(n.getParam("wheel_low_wrap", encoder_low_wrap)){
	 
		ROS_INFO_STREAM("wheel_low_wrap from param" << encoder_low_wrap);	       
	}


        if(n.getParam("wheel_high_wrap", encoder_high_wrap)){
	 
		ROS_INFO_STREAM("wheel_high_wrap from param" << encoder_high_wrap);	       
	}




        if(n.getParam("ticks_meter", ticks_meter)){
	 
		ROS_INFO_STREAM("Ticks meter" << ticks_meter);	       
	}


        if(n.getParam("base_width", base_width )){
	 
		ROS_INFO_STREAM("Base Width" << base_width );	       
	}


/*
	ROS_INFO_STREAM("Encoder min" << encoder_min);
	ROS_INFO_STREAM("Encoder max" << encoder_max);

	ROS_INFO_STREAM("Encoder Low Wrap"<< encoder_low_wrap);
	ROS_INFO_STREAM("Encoder High Wrap" << encoder_high_wrap);

	ROS_INFO_STREAM("ticks meter" << ticks_meter);
	ROS_INFO_STREAM("base width" << base_width);

*/

}

//Update function
void Odometry_calc::publishOdometry(){

	ros::Time now = ros::Time::now();
	
//	ros::Time elapsed;

	double elapsed;

	double d_left, d_right, d, th,x,y;


	if ( now > t_next) {

		elapsed = now.toSec() - then.toSec(); 

 // 	        ROS_INFO_STREAM("elapsed =" << elapsed);

		
		
		if(enc_left == 0){
			d_left = 0;
			d_right = 0;
		}
		else{
			d_left = (left - enc_left) / ( ticks_meter);
			d_right = (right - enc_right) / ( ticks_meter);
		}
		
		enc_left = left;
		enc_right = right;

		d = (d_left + d_right ) / 2.0;

//		ROS_INFO_STREAM("d" << d);


		th = ( d_right - d_left ) / base_width;
		
		dx = d /elapsed;

		dr = th / elapsed;

	
		if ( d != 0){
			x = cos( th ) * d;
			y = -sin( th ) * d;
			// calculate the final position of the robot
			x_final = x_final + ( cos( theta_final ) * x - sin( theta_final ) * y );
			y_final = y_final + ( sin( theta_final ) * x + cos( theta_final ) * y );
		}

		if (th != 0) {
           	theta_final = theta_final + th;
		}

		geometry_msgs::Quaternion odom_quat ;

		odom_quat.x = 0.0;
		odom_quat.y = 0.0;
		odom_quat.z = 0.0;

	    ROS_INFO_STREAM("theta_final =" << theta_final);

		odom_quat.z = sin( theta_final / 2 );	
		odom_quat.w = cos( theta_final / 2 );

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = now;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint";

		odom_trans.transform.translation.x = x_final;
		odom_trans.transform.translation.y = y_final;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
		
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = now;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = x_final;
		odom.pose.pose.position.y = y_final;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_footprint";
		odom.twist.twist.linear.x = dx;
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.angular.z = dr;

		//publish the message
		odom_pub.publish(odom);
   	    then = now;

//		    ROS_INFO_STREAM("dx =" << x_final);

//		    ROS_INFO_STREAM("dy =" << y_final);
		}
	 else { ; }
//		ROS_INFO_STREAM("Not in loop");
}




void Odometry_calc::updateLeftEncoder(const double left_ticks)

{
	if (prev_lencoder == left_ticks) return;

	ROS_INFO_STREAM("Left tick " << (int64_t)left_ticks);
	double enc = left_ticks;

	if((enc < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap))
	{
		
		lmult = lmult + 1;
	}
	

	if((enc > encoder_high_wrap) && (prev_lencoder < encoder_low_wrap))

	{
		
		lmult = lmult - 1;
	}

	left = 1.0 * (enc + lmult * (encoder_max - encoder_min ));

	prev_lencoder = enc;

//	ROS_INFO_STREAM("Left " << left);

}


//Right encoder callback

void Odometry_calc::updateRightEncoder(const double right_ticks)

{
	if (prev_rencoder == right_ticks) return;

	ROS_INFO_STREAM("Right tick " << (int64_t)right_ticks);


	double enc = right_ticks;

	if((enc < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap))
	{
		
		rmult = rmult + 1;
	}
	

	if((enc > encoder_high_wrap) && (prev_lencoder < encoder_low_wrap))

	{
		
		rmult = rmult - 1;
	}

	right = 1.0 * (enc + rmult * (encoder_max - encoder_min ));

	prev_rencoder = enc;

//	ROS_INFO_STREAM("Right " << right);



}

