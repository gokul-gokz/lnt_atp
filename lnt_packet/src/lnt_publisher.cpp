#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <lnt_packet/lnt_packet.h>
#include <sstream>


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "lnt_packet_data");

  ros::NodeHandle n;


 
  ros::Publisher packet_pub = n.advertise<lnt_packet::lnt_packet>("lnt_packet_data", 1000);

  ros::Rate loop_rate(10);// Publishes the message once  per second.

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is the packet data message. You stuff it with data, and then publish it.
     */
    lnt_packet::lnt_packet msg;
	msg.packet_code=1;
	msg.values={-0.05,0,0,0.0,0,0.0};
	msg.eff_mode=1;

    packet_pub.publish(msg);

  
    loop_rate.sleep();
    
  }
  return 0;
}

