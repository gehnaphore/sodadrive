#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

static double const WHEEL_DISTANCE_m = 0.475;
static double const WHEEL_DIAMETER_m = 0.122;
static double const ENCODER_TICKS_PER_SINGLE_MOTOR_REVOLUTION     = 980;

class Odometry_calc{

public:
	Odometry_calc(bool reverse);

	void updateLeftEncoder(const double left_ticks);
	void updateRightEncoder(const double right_ticks);
	void publishOdometry();

private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Subscriber l_wheel_sub;
	ros::Subscriber r_wheel_sub;
	ros::Publisher odom_pub;

	bool mReverse;

	tf::TransformBroadcaster odom_broadcaster;
	//Encoder related variables
	double encoder_min;
	double encoder_max;

	double encoder_low_wrap;
	double encoder_high_wrap;

	double prev_lencoder;
	double prev_rencoder;

	double lmult;
	double rmult;

	double left;
	double right;

	double rate;

	ros::Duration t_delta;

	ros::Time t_next;

	ros::Time then;


	double enc_left ;

	double enc_right;

	double ticks_meter;

	double base_width;

	double dx;

	double dr;

	double x_final,y_final, theta_final;

	ros::Time current_time, last_time;



	void init_variables();

	void get_node_params();

};
