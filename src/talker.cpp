/**
 * @file talker.cpp
 * @author Pradeep Gopal
 * @copyright MIT License
 * @brief Implementing the publisher
 * This demonstrates the simple sending of messages over the ROS system.
 */

/**
 *MIT License

 *Copyright (c) 2020 Pradeep Gopal

 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:

 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.

 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../include/talker.hpp"
#include "beginner_tutorials/changeBaseString.h"

/**
 * Default String message which can be modified via service
 */
DefaultString def;

/**
 * @brief  Callback function for changeString Service
 * @param  req   Request data sent to service
 * @param  res   Response by the service to the client
 * @return bool
 */
bool newMessage(beginner_tutorials::changeBaseString::Request &req,
                beginner_tutorials::changeBaseString::Response &res) {
  def.default_msg = req.inputString;
  ROS_WARN_STREAM("The user changed the string");
  res.newString = req.inputString;
  return true;
}

/**
 * @brief  Broadcaster Function to broadcast a tf frame called /talk with parent /world
 * @param  none
 * @return none
 */
void tfBroadcaster() {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(cos(ros::Time::now().toSec()),
                                    sin(ros::Time::now().toSec()), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 1);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,
                                          ros::Time::now(), "world", "talk"));
}

/**
 * @brief main function
 * @param integer (argc) and character (argv)
 * @return 0
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROSsystem.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);
  auto server = n.advertiseService("changeBaseString", newMessage);

  double my_ratee;

  /**
   * Getting the rate parameter "my_rate" from the parameter server
   */
  n.getParam("/my_rate", my_ratee);

  if (my_ratee <= 500 && my_ratee > 0) {
    ROS_DEBUG_STREAM("Loop Rate Received from "
        "roslaunch argument is " << my_ratee);
  } else if (my_ratee > 500) {
    ROS_ERROR_STREAM("Loop Rate is too high");
    ROS_WARN_STREAM("Setting the loop rate to default value of 10 Hz");
    my_ratee = 10;
  } else if (my_ratee < 0) {
    ROS_ERROR_STREAM("Loop Rate cannot be negative");
    ROS_WARN_STREAM("Setting the loop rate to default value of 10 Hz");
    my_ratee = 10;
  } else if (my_ratee == 0) {
    ROS_FATAL_STREAM("Loop Rate cannot be zero");
    ROS_WARN_STREAM("Setting the loop rate to default value of 10 Hz");
    my_ratee = 10;
  }

  /**
   * Setting the loop rate from the paramter "my_rate" from the parameter server
   */
  ros::Rate loop_rate(my_ratee);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::string msg_data;
    msg_data = def.default_msg + std::to_string(count);
    msg.data = msg_data;
    ROS_INFO_STREAM(msg_data);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    // Calling the Broadcaster Function to broadcast a tf frame called /talk
    // with parent /world
    tfBroadcaster();

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
