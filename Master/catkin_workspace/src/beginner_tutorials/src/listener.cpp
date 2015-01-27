#define DESIRED_DEPTH 0.75
#define DEPTH_TOLERANCE 0.10
#define DEPTH_KP -10
#define DEPTH_THURST_OFFSET -20
#define TIME_OUT 30
#define HOR_THRUST 40
#define THRUST_LIMIT 70

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"

float heading;
float depth;

int clamp(int v, int minv, int maxv)
{
    return std::min(maxv,std::max(minv, v));
}


void CompassCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  //ROS_INFO("x: [%f]", msg->x);
  //ROS_INFO("y: [%f]", msg->y);
  //ROS_INFO("theta: [%f]", msg->theta);
  heading = msg->theta;
}

void SounderCallback(const std_msgs::Float32::ConstPtr& msg)
{
  //ROS_INFO("depth: [%2.1f]", msg->data);
  depth = msg->data;
}


int main(int argc, char **argv)
{


  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  std_msgs::Int16 thrust_hl, thrust_hr, thrust_vl, thrust_vr;
  int count = 0;

  ros::Subscriber sub_compass = n.subscribe("SpartonCompass", 1000, CompassCallback);
  ros::Subscriber sub_sounder = n.subscribe("DepthSounder", 1000, SounderCallback);
  ros::Publisher pub_hl = n.advertise<std_msgs::Int16>("thrust_hl", 500);
  ros::Publisher pub_hr = n.advertise<std_msgs::Int16>("thrust_hr", 500);
  ros::Publisher pub_vl = n.advertise<std_msgs::Int16>("thrust_vl", 500);
  ros::Publisher pub_vr = n.advertise<std_msgs::Int16>("thrust_vr", 500);
  


  ros::Rate loop_rate(10);

  //ros::spin();

  while (ros::ok()) {


    ros::spinOnce();

      // Depth Control
      float depth_E = depth - DESIRED_DEPTH;
      
      ROS_INFO("Depth Error: [%2.1f]", depth_E);

      if ((depth_E*depth_E) > (DEPTH_TOLERANCE*DEPTH_TOLERANCE)) 
      {
          thrust_vr.data=thrust_vl.data = clamp((DEPTH_KP * depth_E) + DEPTH_THURST_OFFSET,-THRUST_LIMIT,THRUST_LIMIT);
          pub_vr.publish(thrust_vr);
          pub_vl.publish(thrust_vl);
          ROS_INFO("Vert Thrust: [%d]", thrust_vr.data);
      }
      else{
       if(count==0) count = 1;
      }

    loop_rate.sleep();


    if(count > 0) 
    {
    	count += 1;
    	if (count > TIME_OUT*12) exit(0);
    	thrust_hr.data=thrust_hl.data=clamp(HOR_THRUST,-THRUST_LIMIT,THRUST_LIMIT);
    	pub_hr.publish(thrust_hr);
        pub_hl.publish(thrust_hl);
    	ROS_INFO("Hor Thrust: [%d]", thrust_hr.data);
    	ROS_INFO("Time elasp: [%d]", count/12);
    	ROS_INFO("**********************************");  
    }
  }



  return 0;
}
