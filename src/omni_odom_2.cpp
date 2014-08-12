#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ax2550/Encoders.h>
#include <tf/transform_datatypes.h>

int wheel1_new, wheel2_new, wheel3_new, wheel4_new;
double dt_front, dt_rear;

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

const double wheel_radius = 0.125;
const double wheel_circumference = 0.785;
const double k = 0.44; //the sum of the distance between the wheel's x-coord and the origin, and the y-coord and the origin
const double encoder_resolution = 1250*4*20;

ros::Publisher odom_pub;
ros::Subscriber fr_enc;
ros::Subscriber rr_enc;
tf::TransformBroadcaster *odom_broadcaster;
void computeOdom();

void feCallBack(const ax2550::Encoders::ConstPtr& msg)
{
  wheel1_new = msg->right_wheel;
  wheel4_new = msg->left_wheel;
  dt_front = msg->time_delta;
}

void reCallBack(const ax2550::Encoders::ConstPtr& msg)
{
  wheel2_new = msg->right_wheel;
  wheel3_new = msg->left_wheel;
  dt_rear = msg->time_delta;
  computeOdom();
}

void computeOdom()
{

  double avg_dt = (dt_front + dt_rear)/2.0;

  if(avg_dt == 0)
  {
      avg_dt = dt_front;
  }

  ros::Time current_time = ros::Time::now();

  double dist_per_tick = wheel_circumference / encoder_resolution;

  //compute the velocities
  double v_w1 = (wheel1_new * dist_per_tick)/dt_front;
  double v_w2 = (wheel2_new * dist_per_tick)/dt_rear;
  double v_w3 = (wheel3_new * dist_per_tick)/dt_rear;
  double v_w4 = (wheel4_new * dist_per_tick)/dt_front;

  vx = (wheel_radius/4)*(v_w1+v_w2+v_w3+v_w4);
  vy = (wheel_radius/4)*(-v_w1+v_w2-v_w3+v_w4);
  vth = (wheel_radius/(4*k))*(-v_w1-v_w2+v_w3+v_w4);

  //compute odometry in a typical way given the velocities of the robot
  double delta_x = vx * avg_dt;
  double delta_y = vy * avg_dt;
  double delta_th = vth * avg_dt;

  x = x + delta_x;
  y = y + delta_y;
  th = th + delta_th;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  odom_broadcaster = new tf::TransformBroadcaster();
  //send the transform
  odom_broadcaster->sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the covariance
  odom.pose.covariance[0] = 0.2;
  odom.pose.covariance[7] = 0.2;
  odom.pose.covariance[14] = 1e100;
  odom.pose.covariance[21] = 1e100;
  odom.pose.covariance[28] = 1e100;
  odom.pose.covariance[35] = 0.2;

  //set the velocity
  odom.child_frame_id = "base_footprint";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  //publish the message
  odom_pub.publish(odom);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "omni_odom");

  ros::NodeHandle n;
  odom_pub = n.advertise<nav_msgs::Odometry>("omni_odom", 60);

  fr_enc = n.subscribe("/front/encoders", 1, feCallBack);
  rr_enc = n.subscribe("/rear/encoders", 1, reCallBack);

  ros::spin();
  return 0;
}
