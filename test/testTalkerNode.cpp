/**
 * @file testTalkerNode.cpp
 * @author Pradeep Gopal
 * @copyright MIT License
 * @brief Tests to test the talker node using g-test framework
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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/changeBaseString.h"
#include "std_msgs/String.h"

/**
 * @brief     Tests whether the changeBaseString service exists or not
 * @param     testTalkerNode        gtest framework
 * @param     testExistence         Name of the test
 */
TEST(testTalkerNode, testExistence) {
// Create the node handle
ros::NodeHandle n;
// Register a client to the changeBaseString service
auto client = n.serviceClient
        <beginner_tutorials::changeBaseString>("changeBaseString");
// Tests if the service exists or not
EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
}

/**
 * @brief     Tests whether the changeBaseString service can modify the output message
 * @param     testTalkerNode           gtest framework
 * @param     testModifyMessage        Name of the test
 */
TEST(testTalker, testModifyMessage) {
// Create the node handle
ros::NodeHandle n;
// Register a client to the changeBaseString service
auto client = n.serviceClient
        <beginner_tutorials::changeBaseString>("changeBaseString");
// Initialize the service to srv object
beginner_tutorials::changeBaseString srv;
// Modify the input string
srv.request.inputString = "testing_string_change";
// Calling the client
client.call(srv.request, srv.response);
// Tests to see if the string is modified or not
EXPECT_STREQ("testing_string_change", srv.response.newString.c_str());
}