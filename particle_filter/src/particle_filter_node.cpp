#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include<math.h>

//Code creates, updates and publishes particles, every 0.5 seconds,
//within a 10mX10m square, with corners at (5m, 5m) and (-5m, -5m)
//at random positions and with random orientations 

int numParticles;

ros::Publisher particlecloud_pub;

geometry_msgs::PoseArray cloud_msg;

void timerCallback(const ros::TimerEvent &)
{   
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "odom";
    
    for (int i = 0; i < numParticles; i++)
    {
        cloud_msg.poses[i].position.x = ((float)rand()/RAND_MAX)*10-5;
        cloud_msg.poses[i].position.y = ((float)rand()/RAND_MAX)*10-5;
        cloud_msg.poses[i].position.z = 0;
        int deg = (((float)rand()/RAND_MAX)*360-180);
        float tmp = deg*M_PI/360;
        //std::cout << "particle orientation:" << deg << " degrees" <<std::endl;
        cloud_msg.poses[i].orientation.w = cos(tmp);
        cloud_msg.poses[i].orientation.x = 0;
        cloud_msg.poses[i].orientation.y = 0;
        cloud_msg.poses[i].orientation.z = sin(tmp);
    }
    particlecloud_pub.publish(cloud_msg);
}

int main(int argc, char **argv)
{

    //you may also run the node with numParticles argument as follows
    //rosrun particle_filter particle_filter_node _numParticles:=10

    ros::init(argc, argv, "particle_filter_node");
    ros::NodeHandle n("~");
    
    if (!(n.getParam("numParticles",numParticles)))
        numParticles = 100;


    std::cout << "numParticles" << numParticles <<std::endl;

    // a timer that cause a callback every 0.5 seconds
    ros::Timer timer = n.createTimer(ros::Duration(0.5), timerCallback);

    particlecloud_pub = n.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);

    cloud_msg.poses.resize(numParticles);


    ros::spin();

    return 0;
}