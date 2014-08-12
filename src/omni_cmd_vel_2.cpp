#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace std;

ros::Publisher front_cmd_vel_pub;
ros::Publisher rear_cmd_vel_pub;

double wheel_circumference = 0.785;
double wheel_base_length = 0.44;
double wheel_diameter = 0.25;
double wheel_radius = 0.125;

// Max speed = 1.3 km/hr
// Limiting speed to 1 km/hr
// That is 21.25 rpm, roughly 96 input

static double REL_MAX = 96.0;

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
  const double k = 44.0; //the sum of the distance between the wheel's x-coord and the origin, and the y-coord and the origin 
	
  // Copy the velocities (m/s)
  double Vx = msg->linear.x;
  double Vy = msg->linear.y;
  double Wv = msg->angular.z;
	
  // Calculate individual wheel velocities (wheel 1 is the front left corner, going counterclockwise)(m/s)
  double W1 = (1 / wheel_radius) * (Vx - Vy + Wv * -k); 
  double W2 = (1 / wheel_radius) * (Vx + Vy + Wv * -k); 
  double W3 = (1 / wheel_radius) * (Vx - Vy + Wv * k); 
  double W4 = (1 / wheel_radius) * (Vx + Vy + Wv * k); 
	
  // Convert from mps to rpm
  double front_A_rpm = W4 * (60.0 / (M_PI*wheel_diameter));
  double front_B_rpm = W1 * (60.0 / (M_PI*wheel_diameter));
  double rear_A_rpm = W3 * (60.0 / (M_PI*wheel_diameter));
  double rear_B_rpm = W2 * (60.0 / (M_PI*wheel_diameter));
    
  // Convert rpm to relative (pg 81)
  double front_A_rel = front_A_rpm * 1250 * 11 / 58593.75; //(rpm * PPR * Time Base+1)/58593.75(from Operating Manual)
  double front_B_rel = front_B_rpm * 1250 * 11 / 58593.75;
  double rear_A_rel = rear_A_rpm * 1250 * 11 / 58593.75;
  double rear_B_rel = rear_B_rpm * 1250 * 11 / 58593.75;

  // Bounds check
  if(front_A_rel > REL_MAX)
  {
    double rel = REL_MAX / front_A_rel;
    front_A_rel = REL_MAX;
	  front_B_rel = front_B_rel * rel;
	  rear_A_rel = rear_A_rel * rel;
	  rear_B_rel = rear_B_rel * rel;
  }
	
  if(front_A_rel < -1*REL_MAX)
  {
	  double rel = REL_MAX / front_A_rel;
	  front_A_rel = -1 * REL_MAX;
	  front_B_rel = front_B_rel * rel;
	  rear_A_rel = rear_A_rel * rel;
	  rear_B_rel = rear_B_rel * rel;
  }
	
  if(front_B_rel > REL_MAX)
  {
    double rel = REL_MAX / front_B_rel;
    front_A_rel = front_A_rel * rel;
    front_B_rel = REL_MAX;
    rear_A_rel = rear_A_rel * rel;
	  rear_B_rel = rear_B_rel * rel;
  }
	
  if(front_B_rel < -1*REL_MAX)
  {
	  double rel = REL_MAX / front_B_rel;
    front_A_rel = front_A_rel * rel;
    front_B_rel = -1 * REL_MAX;
    rear_A_rel = rear_A_rel * rel;
    rear_B_rel = rear_B_rel * rel;
  }
	
if(rear_A_rel > REL_MAX)
{
    double rel = REL_MAX / rear_A_rel;
    front_A_rel = front_A_rel * rel;
	  front_B_rel = front_B_rel * rel;
	  rear_A_rel = REL_MAX;
	  rear_B_rel = rear_B_rel * rel;
  }
	
  if(rear_A_rel < -1*REL_MAX)
  {
	  double rel = REL_MAX / rear_A_rel;
	  front_A_rel = front_A_rel * rel;
	  front_B_rel = front_B_rel * rel;
	  rear_A_rel = -1 * REL_MAX;
	  rear_B_rel = rear_B_rel * rel;
  }
	
  if(rear_B_rel > REL_MAX)
  {
	  double rel = REL_MAX / rear_B_rel;
	  front_A_rel = front_A_rel * rel;
	  front_B_rel = front_B_rel * rel;
	  rear_A_rel = rear_A_rel * rel;
	  rear_B_rel = REL_MAX;
  }
	
  if(rear_B_rel < -1*REL_MAX)
  {
	  double rel = REL_MAX / rear_B_rel;
	  front_A_rel = front_A_rel * rel;
	  front_B_rel = front_B_rel * rel;
	  rear_A_rel = rear_A_rel * rel;
	  rear_B_rel = -1 * REL_MAX;
  }

  // publish motor speeds
  geometry_msgs::Twist front_cmd_vel;
  front_cmd_vel.linear.x = front_A_rel * -1;
  front_cmd_vel.linear.y = front_B_rel * -1;
  front_cmd_vel.linear.z = 0.0;
  front_cmd_vel.angular.x = 0.0;
  front_cmd_vel.angular.y = 0.0;
  front_cmd_vel.angular.z = 0.0;

  geometry_msgs::Twist rear_cmd_vel;
  rear_cmd_vel.linear.x = rear_A_rel;
  rear_cmd_vel.linear.y = rear_B_rel;
  rear_cmd_vel.linear.z = 0.0;
  rear_cmd_vel.angular.x = 0.0;
  rear_cmd_vel.angular.y = 0.0;
  rear_cmd_vel.angular.z = 0.0;
  
  front_cmd_vel_pub.publish(front_cmd_vel);
  rear_cmd_vel_pub.publish(rear_cmd_vel);
}

int main(int argc, char **argv) 
{ 
  // Node setup
  ros::init(argc, argv, "omni_cmd_vel");
  ros::NodeHandle n;

  // cmd_vel publishers
  front_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/front/cmd_vel", 5);
  rear_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/rear/cmd_vel", 5);

  // cmd_vel Subscriber
  ros::Subscriber sub = n.subscribe("cmd_vel", 5, cmd_velCallback);

  ros::spin();
  
  return 0;
}
