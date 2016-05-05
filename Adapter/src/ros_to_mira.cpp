/*
 * Copyright (C) 2012 by
 *   MetraLabs GmbH (MLAB), GERMANY
 * and
 *   Neuroinformatics and Cognitive Robotics Labs (NICR) at TU Ilmenau, GERMANY
 * All rights reserved.
 *
 * Contact: info@mira-project.org
 *
 * Commercial Usage:
 *   Licensees holding valid commercial licenses may use this file in
 *   accordance with the commercial license agreement provided with the
 *   software or, alternatively, in accordance with the terms contained in
 *   a written agreement between you and MLAB or NICR.
 *
 * GNU General Public License Usage:
 *   Alternatively, this file may be used under the terms of the GNU
 *   General Public License version 3.0 as published by the Free Software
 *   Foundation and appearing in the file LICENSE.GPL3 included in the
 *   packaging of this file. Please review the following information to
 *   ensure the GNU General Public License version 3.0 requirements will be
 *   met: http://www.gnu.org/copyleft/gpl.html.
 *   Alternatively you may (at your option) use any later version of the GNU
 *   General Public License if such license has been publicly approved by
 *   MLAB and NICR (or its successors, if any).
 *
 * IN NO EVENT SHALL "MLAB" OR "NICR" BE LIABLE TO ANY PARTY FOR DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF
 * THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF "MLAB" OR
 * "NICR" HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * "MLAB" AND "NICR" SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND "MLAB" AND "NICR" HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS OR MODIFICATIONS.
 */

/**
 * @file ros_to_mira.cpp
 *    an adapter that transforms and publishes OccupancyGrids from ROS to MIRA
 *
 * @author Dirk Steindorf
 * @date   2015/07/14
 */

#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <fw/Framework.h>
#include <fw/ChannelReadWrite.h>
#include <platform/Types.h>
#include <transform/Pose.h>
#include <transform/Velocity.h>
#include <robot/RangeScan.h>

using namespace std;
using namespace mira;



//-----------------------------------------------------------------------------
// MIRA-specific stuff
mira::Authority authority; 

// channel for publishing a ROS OccupancyGrid to MIRA
//mira::Channel<GridMap<uint8>> ros_grid_to_mira;

//-----------------------------------------------------------------------------
// ROS-specific stuff
ros::Publisher scitosOdometryPub;
ros::Publisher scitosLaserPub;


// callback for sending odometry to ROS
void onNewOdometry(mira::ChannelRead<mira::Pose2> data)
{
    nav_msgs::Odometry msg;
    mira::Pose2 pose = data->value();

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odometry_frame";

    msg.pose.pose.position.x = (float)pose.x();
    msg.pose.pose.position.y = (float)pose.y();
    msg.twist.twist.angular.x = (float)pose.phi();
    
    scitosOdometryPub.publish(msg);
}

// callback for sending laserscanner data to ROS

void onNewLaser(mira::ChannelRead<robot::RangeScan> data)
{
    sensor_msgs::LaserScan msg;
    robot::RangeScan rangeScan = data->value();

    msg.header.stamp.sec = ros::Time::now();
    msg.header.frame_id = "sick_frame";

    msg.angle_min = 0.0;
    msg.angle_max = 270.0;
    msg.angle_increment = 0.5;

    msg.scan_time = (float) rangeScan.scanTime.seconds();

    msg.range_min = 0.0;
    msg.range_max = 2.0;

    msg.ranges = rangeScan.range;

    scitosLaserPub.publish(msg);
}


// callback for velocity commands to the SCITOS
void vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    authority.callService<void>("/robot/Robot", "setVelocity", 
        Velocity2(msg->linear.x, 0.0f, msg->angular.x));
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Ros2Mira");

    mira::Framework framework(argc, argv, true);

    authority.checkin("/", "Ros2Mira");
    authority.start();

    // subscribe to MIRA
    authority.subscribe<mira::Pose2>("/robot/Odometry", &onNewOdometry);
    authority.subscribe<robot::RangeScan>("/robot/frontLaser/Laser", &onNewLaser);


    // ROS publisher
    ros::NodeHandle pubNode1;
    ros::NodeHandle pubNode2;

    scitosOdometryPub = pubNode1.advertise<geometry_msgs::Point>("mira_odometry", 1000);
    scitosLaserPub = pubNode2.advertise<sensor_msgs::LaserScan>("scan", 1000);

    // ROS subscriber
    ros::NodeHandle subNode1;

    ros::Subscriber subscriber1 = subNode1.subscribe("velocity_cmd", 1000, vel_callback);
}
